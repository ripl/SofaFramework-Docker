
import random
import numpy as np
from scipy.spatial.transform import Rotation
import time


def set_seed(seed=0):
    import random
    import numpy as np
    import tensorflow as tf
    import torch
    random.seed(seed)
    np.random.seed(seed)
    tf.random.set_random_seed(seed)
    torch.manual_seed(0)
