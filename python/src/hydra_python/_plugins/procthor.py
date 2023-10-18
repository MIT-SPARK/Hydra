# %%
import matplotlib.pyplot as plt
import numpy as np
import ai2thor
import ai2thor.controller

# import ai2thor.platform
import prior


# %%
dataset = prior.load_dataset("procthor-10k")


# %%
house = dataset["train"][3]


# %%
controller = ai2thor.controller.Controller(
    scene=house,
    renderDepthImage=True,
    renderInstanceSegmentation=True,
    width=640,
    height=480,
    fieldOfView=70,
)


# %%
fig, ax = plt.subplots(1, 3)
event = controller.last_event
rgb = np.asarray(event.frame)
depth = np.asarray(event.depth_frame)
semantics = np.asarray(event.instance_segmentation_frame)
ax[0].imshow(rgb)
ax[1].imshow(depth)
ax[2].imshow(semantics)
plt.show()
