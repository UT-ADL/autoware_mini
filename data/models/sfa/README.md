This directory contains SFA models trained on different
datasets and under different conditions: 

1. Nu = Nuscenes dataset.
2. Ly = Lyft dataset.
3. Ar = Argoverse1 dataset.
4. On = Once dataset.
5. Ar2 = Argoverse2 dataset.

The naming convention adopted is as follows: 

**datasets_xx_epochs.onnx**

where 

- `datasets` corresponds to one or a combination of
datasets listed above.
- `01` for xx correponds to short range while `02` stands for long range model.
- `epochs` is the number of epochs the model was trained for.

### Params for long and short range models

Short range params:
- BEV_HEIGHT = 608  
- BEV_WIDTH = 608  
- MIN_FRONT_X = 0  
- MAX_FRONT_X = 50 
- MIN_BACK_X = -50 
- MAX_BACK_X = 0  
- MIN_Y = -25  
- MAX_Y = 25  

Long range params:
- BEV_HEIGHT = 1216  
- BEV_WIDTH = 1216  
- MIN_FRONT_X = 0  
- MAX_FRONT_X = 80 
- MIN_BACK_X = -80 
- MAX_BACK_X = 0  
- MIN_Y = -40  
- MAX_Y = 40  