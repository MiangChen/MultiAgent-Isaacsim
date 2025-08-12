#!/usr/bin/env python

import cv2
import logging
import numpy as np
import time

import carb
import tensorrt as trt
import torch


class FisheyeTRTProcessor:
    def __init__(self, engine_file, feat_coords_file, overlap_mask_file):

        self.num_cameras = 7
        self.visualize = False
        self.save_images = False
        self.save_images_path = "/tmp/stacked_images.npy"

        # Initialize loggers
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        console_handler = logging.StreamHandler()
        formatter = logging.Formatter("%(name)s - %(levelname)s - %(message)s")
        console_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)

        self.trt_logger = trt.Logger(trt.Logger.INFO)

        self.device = torch.device("cuda")

        try:
            # Load TensorRT engine
            with open(engine_file, "rb") as f:
                runtime = trt.Runtime(self.trt_logger)
                self.engine = runtime.deserialize_cuda_engine(f.read())
                self.trt_context = self.engine.create_execution_context()

            self.logger.info(f"TensorRT engine loaded successfully from {engine_file}")

        except Exception as e:
            self.logger.error(f"Failed to load TensorRT engine from {engine_file}: {e}")
            raise e

        trt_to_torch_dtype_dict = {
            trt.bool: torch.bool,
            trt.uint8: torch.uint8,
            trt.int8: torch.int8,
            trt.int32: torch.int32,
            trt.int64: torch.int64,
            trt.float16: torch.float16,
            trt.float32: torch.float32,
        }
        # Get shapes and dtypes for inputs and outputs
        self.input_desc = []
        self.output_desc = []

        for idx in range(self.engine.num_io_tensors):
            tensor_name = self.engine.get_tensor_name(idx)
            desc = (
                tensor_name,
                self.engine.get_tensor_shape(tensor_name),
                trt_to_torch_dtype_dict[self.engine.get_tensor_dtype(tensor_name)],
            )
            if self.engine.get_tensor_mode(tensor_name) == trt.TensorIOMode.INPUT:
                self.input_desc.append(desc)
            else:
                self.output_desc.append(desc)

        # Load feat_coords_lut
        feat_coords_lut = torch.from_numpy(np.load(feat_coords_file))
        self.logger.info(
            f"Using feat_coords from {feat_coords_file} with shape: {feat_coords_lut.shape}"
        )
        # Check if the feat_coords_lut matches the engine input shape
        if feat_coords_lut.shape != self.input_desc[1][1]:
            self.logger.error(
                f"feat_coords_lut shape {feat_coords_lut.shape} does not match the engine input shape {self.input_desc[1][1]}"
            )
            raise ValueError(
                "feat_coords_lut shape does not match the engine input shape"
            )

        # Copy feat_coords_lut into GPU memory and set the second input tensor address
        self.feat_coords_lut_gpu = feat_coords_lut.to(
            device=self.device, dtype=self.input_desc[1][2]
        )
        self.trt_context.set_tensor_address(
            self.input_desc[1][0], int(self.feat_coords_lut_gpu.data_ptr())
        )

        if len(self.input_desc) > 2:
            # Load overlap_mask
            overlap_mask = torch.from_numpy(np.load(overlap_mask_file))
            self.logger.info(
                f"Using overlap_mask from {overlap_mask_file} with shape: {overlap_mask.shape}"
            )
            # Check if the overlap_mask matches the engine input shape
            if overlap_mask.shape != self.input_desc[2][1]:
                self.logger.error(
                    f"overlap_mask shape {overlap_mask.shape} does not match the engine input shape {self.input_desc[2][1]}"
                )
                raise ValueError(
                    "overlap_mask shape does not match the engine input shape"
                )

            # Copy overlap_mask into GPU memory and set the third input tensor address
            self.overlap_mask_gpu = overlap_mask.to(
                device=self.device, dtype=self.input_desc[2][2]
            )
            self.trt_context.set_tensor_address(
                self.input_desc[2][0], int(self.overlap_mask_gpu.data_ptr())
            )

        # Allocate output tensors
        self.outputs = []
        for desc in self.output_desc:
            self.outputs.append(
                torch.empty(tuple(desc[1]), dtype=desc[2], device=self.device)
            )
            # Set output tensor address
            self.trt_context.set_tensor_address(
                desc[0], int(self.outputs[-1].data_ptr())
            )

        self.depth_pred = np.empty(tuple(self.output_desc[0][1]), dtype=np.float32)

    @carb.profiler.profile
    def process_images(self, images):
        # Check if the images tensor is of the correct format (N, H, W, C)
        if (
            images.ndim != 4
            or images.shape[0] != self.num_cameras
            or images.shape[3] != 3
        ):
            self.logger.warning(
                f"Received images tensor shape is wrong: {images.shape}"
            )
            return None

        if self.visualize:
            self.visualize_images(images)

        if self.save_images:
            # save the images as .npy file
            np.save(self.save_images_path, images.cpu().numpy())
            self.save_images = False
            self.logger.info("Stacked images saved to stacked_images.npy")

        if not self.run_inference(images):
            return None

        return self.depth_pred

    @carb.profiler.profile
    def run_inference(self, images):
        try:
            time_start = time.time()

            # images_f32 = images.to(device=self.device, dtype=torch.float32)
            # Convert images to BGR format and float32
            images_f32 = torch.flip(images, dims=[3]).to(
                device=self.device, dtype=torch.float32
            )
            expected_size = trt.volume(self.input_desc[0][1]) * trt.float32.itemsize
            if images_f32.nbytes != expected_size:
                self.logger.warning(
                    f"images size is wrong: {images_f32.nbytes} != {expected_size}"
                )
                return False

            # Normalize images to [-1, 1]
            images_f32 = images_f32.mul_(2.0 / 255).sub_(1)

            # Convert from (N, H, W, C) to (N, C, H, W)
            images_f32 = images_f32.permute((0, 3, 1, 2))

            # Get a contiguous array to pass to the model
            images_f32 = images_f32.ravel()

            # Set input image data address
            self.trt_context.set_tensor_address(
                self.input_desc[0][0], int(images_f32.data_ptr())
            )

            # Run inference with default CUDA stream
            self.trt_context.execute_async_v3(
                stream_handle=torch.cuda.default_stream().cuda_stream
            )

            # Transfer predictions back from the GPU.
            torch.from_numpy(self.depth_pred).copy_(self.outputs[0], non_blocking=False)

            time_end = time.time()
            self.logger.info(
                f"TensorRT inference time: {(time_end - time_start)} seconds"
            )
            return True
        except Exception as e:
            self.logger.error(f"Error during inference: {e}")
            return False

    @carb.profiler.profile
    def visualize_images(self, images):
        display_images_list = []
        for i in range(images.shape[0]):
            img_small = cv2.resize(images[i].cpu().numpy(), (0, 0), fx=0.3, fy=0.3)
            # add cam idx to the image
            cv2.putText(
                img_small,
                f"cam{i}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2,
            )
            display_images_list.append(img_small)

        # Concatenate images horizontally for a single window display
        if display_images_list:
            concatenated_display_image = cv2.hconcat(display_images_list)
            cv2.imshow("Stacked Camera Feeds", concatenated_display_image)
            cv2.waitKey(1)  # Use a small delay (e.g., 1 ms) for continuous display
        else:
            self.logger.warning("No images to display.")
