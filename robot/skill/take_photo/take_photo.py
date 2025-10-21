def take_photo(self, file_path: str = None):
    if self.camera is not None:
        rgb = self.camera.get_rgb()
        if rgb != None and file_path is not None:
            self.camera.save_rgb_to_file(rgb_tensor_gpu=rgb, file_path=file_path)
        return rgb
    else:
        return None
