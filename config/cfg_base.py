from pydantic import BaseModel


class CfgBase(BaseModel):
    def update(self, **kwargs):
        return self.model_copy(update=kwargs, deep=True)
