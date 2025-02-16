# Anim Transfer (gltf)
![main](images/main.png)

Transfer skeletal animations between human-like models (i.e. retargeting). Based on Unity's Mecanim humanoids and Unreal retargetters.

This is useful for those building custom 3d game engines and would like to use existing animation assets from the popular marketplaces.

The tool is written in C with an OpenGL frontend.

## Work in progress
Reads a .gltf source animation and target skeleton and produces a visualization of a reasonable transfer, but does not yet export to any format.

## Usage
(Not yet...)

Will eventually support a full UI-based workflow aswell as a commandline interface to support integration into animation pipelines.

Should end up looking something like this:
```shell
animtransfer.bat -input animation.gltf -target skeleton.gltf -out retargetted_animation.gltf
```

## Building
```
vcvarsall.bat x64
build.bat <debug|release> <run>
```

## How it works
### 1. Standard human bone hierarchy
Input source skeletons must be in a standard bone hierarchy setup.
A mapping file must be provided to identify the individual bones (a simple .txt)

The code looks something like this:
```
typedef enum HumanJoint {
    ...
	HumanJoint_Root,
	HumanJoint_Pelvis,
	HumanJoint_ClavicleL,
	HumanJoint_ClavicleR,
	... 
}
```

The jointmap file looks something like this for a typical Epic rig:
(where <standard_human_joint_name>=<joint_name_in_gltf_file>)
```
root=root
pelvis=pelvis
spine01=spine_01
spine02=spine_02
spine03=spine_03
neck=neck_01
head=head
claviclel=clavicle_l
clavicler=clavicle_r
```

### 2. Bind pose matching
Before we can retarget from skeleton A to skeleton B, we have to match the bind poses.

Similar to existing retargetters, the best setup is a full T-pose. The builtin matcher will do a decent job of matching the pose
and can be visualized+tweaked.

### 3. Transfer animation
Each keyframe is retargetted onto the target skeleton and saved. There are a couple of options we can use to tweak the final output.

### 4. IK
There is an extra IK stage to make sure the hands and feet end up in the correct locations. Can be toggled on or off.

### 5. Final output
Export the freshly baked retargetted animation to .gltf

## Dependencies
- Nuklear IMGUI
- GLFW
- GLAD
- cglm