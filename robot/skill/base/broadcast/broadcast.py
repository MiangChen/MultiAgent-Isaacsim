from typing import Dict, Any


def broadcast(self, params: Dict[str, Any]):
    content = params.get("content")
    logger.info(
        f"[Skill] {self.body.cfg_robot.name} executing broadcasting. The content is {content}"
    )
    return True
