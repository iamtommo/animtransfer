#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include "cglm/cglm.h"

#define CGLTF_IMPLEMENTATION
#define CGLTF_WRITE_IMPLEMENTATION
#include "cgltf_write.h"

#define GLAD_GL_IMPLEMENTATION
#include <glad/gl.h>
#include <GLFW/glfw3.h>

#define NK_INCLUDE_FIXED_TYPES
#define NK_INCLUDE_STANDARD_IO
#define NK_INCLUDE_STANDARD_VARARGS
#define NK_INCLUDE_DEFAULT_ALLOCATOR
#define NK_INCLUDE_VERTEX_BUFFER_OUTPUT
#define NK_INCLUDE_FONT_BAKING
#define NK_INCLUDE_DEFAULT_FONT
#define NK_IMPLEMENTATION
#define NK_GLFW_GL3_IMPLEMENTATION
#define NK_KEYSTATE_BASED_INPUT
#include "include/nuklear.h"
#include "include/nuklear_glfw_gl3.h"

#ifdef _WIN32
#include <string.h>
#define strcasecmp _stricmp
#else
#include <strings.h>
#endif

typedef struct Global {
	float cameraYaw;
	float cameraPitch;
	float cameraDistance;
	bool leftMouseDown;
	double lastMouseX;
	double lastMouseY;
	int fps;
	bool animation_playing;
	float playback_rate;
} Global;

static Global global = {
		.cameraYaw = 0.0f,
		.cameraPitch = 20.0f,
		.cameraDistance = 6.0f,
		.leftMouseDown = false,
		.lastMouseX = 0.0,
		.lastMouseY = 0.0,
		.animation_playing = true,
		.playback_rate = 1.0f
};

typedef enum Mode {
	Mode_Default,
	Mode_Pose,
	Mode_IK
} Mode;

static Mode mode = Mode_IK;

static struct nk_context *nk_ctx;

enum PelvisMotionSpace {
	PelvisMotionSpace_Source = 0,
	PelvisMotionSpace_CenterOfMass = 1,
	PelvisMotionSpace_Feet = 2,
	PelvisMotionSpace_End = 3
};

const char* PelvisMotionSpaceName[] = {
		"Source",
		"Center of Mass",
		"Feet"
};

typedef struct RetargetConfig {
    int auto_tpose;
	int allow_root_motion;
	int allow_pelvis_motion;
    int scale_pelvis_motion_to_target;
	enum PelvisMotionSpace pelvis_motion_space;
	int enable_hand_ik;
	int enable_foot_ik;
} RetargetConfig;

static struct RetargetConfig retarget_config = {
        .auto_tpose = 1,
		.allow_root_motion = 1,
		.allow_pelvis_motion = 1,
        .scale_pelvis_motion_to_target = 1,
		.pelvis_motion_space = PelvisMotionSpace_Source,
		.enable_hand_ik = 1,
		.enable_foot_ik = 1
};

static GLFWwindow* window;

/* ARENAS */
typedef struct Arena {
	char *memory;
	size_t size;
	size_t offset;
} Arena;

Arena* arena_create(size_t size) {
	Arena *arena = malloc(sizeof(Arena));

	arena->memory = malloc(size);
	if (arena->memory == NULL) {
		fprintf(stderr, "arena_create failed\n");
		free(arena);
		exit(1);
	}

	arena->size = size;
	arena->offset = 0;

	return arena;
}

void arena_free(Arena *arena) {
	free(arena->memory);
	free(arena);
}

void* arena_alloc(Arena *arena, size_t size) {
	if (arena->size - arena->offset < size) {
		fprintf(stderr, "Arena out of memory\n");
		exit(1);
	}
	void *ptr = arena->memory + arena->offset;
	arena->offset = arena->offset + size;
	return ptr;
}

void* arena_alloc_zeroed(Arena *arena, size_t size) {
	if (arena->size - arena->offset < size) {
		fprintf(stderr, "Arena out of memory\n");
		exit(1);
	}
	memset(arena->memory + arena->offset, 0, size);
	void *ptr = arena->memory + arena->offset;
	arena->offset = arena->offset + size;
	return ptr;
}

static Arena *global_arena;
/* ARENAS */

#define MAX_VERTEX_BUFFER 512 * 1024
#define MAX_ELEMENT_BUFFER 128 * 1024

static const char* vertex_shader_src =
		"#version 330 core\n"
		"layout(location = 0) in vec3 aPos;\n"
		"uniform mat4 MVP;\n"
		"void main(){\n"
		"    gl_Position = MVP * vec4(aPos, 1.0);\n"
		"}";

static const char* fragment_shader_src =
		"#version 330 core\n"
		"out vec4 FragColor;\n"
		"void main() {\n"
		"   FragColor = vec4(0.2, 0.7, 0.3, 1.0);\n"
		"}";

static const char* line_vertex_shader_src =
		"#version 330 core\n"
		"layout(location = 0) in vec3 aPos;\n"
		"layout(location = 1) in vec3 aColor;\n"
		"uniform mat4 MVP;\n"
		"out vec3 vColor;\n"
		"void main() {\n"
		"	vColor = aColor;\n"
		"	gl_Position = MVP * vec4(aPos, 1.0);\n"
		"}\n";

static const char* line_fragment_shader_src =
		"#version 330 core\n"
		"in vec3 vColor;\n"
		"out vec4 FragColor;\n"
		"void main() {\n"
		"    FragColor = vec4(vColor, 1.0);\n"
		"}\n";

typedef struct {
	float *vertices; // {x, y, z, r, g, b}
	size_t capacity;
	size_t count; // number of floats
	GLuint line_prog;
	GLuint line_vao;
	GLuint line_vbo;
	GLint line_mvp_loc;
} LineBuffer;

static LineBuffer line_buffer = {0};

void draw_line(vec3 start, vec3 end, vec3 color) {
	// 12 floats per line
	// v1: x,y,z, r,g,b | v2: x,y,z, r,g,b

	if (line_buffer.count + 12 > line_buffer.capacity) {
		line_buffer.capacity *= 2;
		line_buffer.vertices = (float*)realloc(line_buffer.vertices,
											   sizeof(float) * line_buffer.capacity);
	}

	float* v = line_buffer.vertices + line_buffer.count;

	v[0] = start[0]; v[1] = start[1]; v[2] = start[2];
	v[3] = color[0]; v[4] = color[1]; v[5] = color[2];

	v[6]  = end[0];  v[7]  = end[1];  v[8]  = end[2];
	v[9]  = color[0]; v[10] = color[1]; v[11] = color[2];

	line_buffer.count += 12;
}

void draw_line_dashed(vec3 start, vec3 end, vec3 color,float dashLength, float gapLength) {
	vec3 dir;
	glm_vec3_sub(end, start, dir);
	float totalLen = glm_vec3_norm(dir);

	if (totalLen < 1e-6f) {
		/* start and end are almost the same, just do a tiny line or skip */
		draw_line(start, end, color);
		return;
	}

	glm_vec3_scale(dir, 1.0f/totalLen, dir);

	float step = dashLength + gapLength;
	float currentDist = 0.0f;

	while (currentDist < totalLen) {
		float dashStart = currentDist;
		float dashEnd   = dashStart + dashLength;

		if (dashEnd > totalLen) {
			dashEnd = totalLen;
		}

		vec3 segStart;
		glm_vec3_copy(start, segStart);
		glm_vec3_scale(dir, dashStart, segStart);
		glm_vec3_add(start, segStart, segStart);

		vec3 segEnd;
		glm_vec3_copy(start, segEnd);
		glm_vec3_scale(dir, dashEnd, segEnd);
		glm_vec3_add(start, segEnd, segEnd);

		draw_line(segStart, segEnd, color);

		currentDist += step;
	}
}

void draw_sphere(vec3 center, float radius, int slices, int stacks, vec3 color) {
	if (slices < 2) slices = 2;
	if (stacks < 2) stacks = 2;

	float deltaPhi   = (float)(M_PI / (float)stacks);
	float deltaTheta = (float)((2.0 * M_PI) / (float)slices);

	for (int i = 1; i < stacks; i++){
		float phi = -M_PI / 2.0f + (float)i * deltaPhi;
		float cosPhi = cosf(phi);
		float sinPhi = sinf(phi);

		for (int j = 0; j < slices; j++){
			float theta1 = j * deltaTheta;
			float theta2 = (j + 1) * deltaTheta;

			vec3 p1 = {
					center[0] + radius * cosPhi * cosf(theta1),
					center[1] + radius * sinPhi,
					center[2] + radius * cosPhi * sinf(theta1)
			};
			vec3 p2 = {
					center[0] + radius * cosPhi * cosf(theta2),
					center[1] + radius * sinPhi,
					center[2] + radius * cosPhi * sinf(theta2)
			};
			draw_line(p1, p2, color);
		}
	}

	for (int j = 0; j < slices; j++){
		float theta = j * deltaTheta;

		for (int i = 0; i < stacks; i++){
			float phi1 = -M_PI / 2.0f + i * deltaPhi;
			float phi2 = -M_PI / 2.0f + (i + 1) * deltaPhi;

			float cosPhi1 = cosf(phi1);
			float sinPhi1 = sinf(phi1);
			float cosPhi2 = cosf(phi2);
			float sinPhi2 = sinf(phi2);

			vec3 p1 = {
					center[0] + radius * cosPhi1 * cosf(theta),
					center[1] + radius * sinPhi1,
					center[2] + radius * cosPhi1 * sinf(theta)
			};
			vec3 p2 = {
					center[0] + radius * cosPhi2 * cosf(theta),
					center[1] + radius * sinPhi2,
					center[2] + radius * cosPhi2 * sinf(theta)
			};
			draw_line(p1, p2, color);
		}
	}
}


static float cube_vertices[] = {
		// front face
		-0.5f,-0.5f, 0.5f,
		0.5f,-0.5f, 0.5f,
		0.5f, 0.5f, 0.5f,
		0.5f, 0.5f, 0.5f,
		-0.5f, 0.5f, 0.5f,
		-0.5f,-0.5f, 0.5f,

		// back face
		-0.5f,-0.5f,-0.5f,
		-0.5f, 0.5f,-0.5f,
		0.5f, 0.5f,-0.5f,
		0.5f, 0.5f,-0.5f,
		0.5f,-0.5f,-0.5f,
		-0.5f,-0.5f,-0.5f,

		// left face
		-0.5f, 0.5f, 0.5f,
		-0.5f, 0.5f,-0.5f,
		-0.5f,-0.5f,-0.5f,
		-0.5f,-0.5f,-0.5f,
		-0.5f,-0.5f, 0.5f,
		-0.5f, 0.5f, 0.5f,

		// right face
		0.5f, 0.5f,  0.5f,
		0.5f,-0.5f, -0.5f,
		0.5f, 0.5f, -0.5f,
		0.5f,-0.5f, -0.5f,
		0.5f, 0.5f,  0.5f,
		0.5f,-0.5f,  0.5f,

		// bottom face
		-0.5f,-0.5f,-0.5f,
		0.5f,-0.5f,-0.5f,
		0.5f,-0.5f, 0.5f,
		0.5f,-0.5f, 0.5f,
		-0.5f,-0.5f, 0.5f,
		-0.5f,-0.5f,-0.5f,

		// top face
		-0.5f,0.5f,-0.5f,
		0.5f,0.5f, 0.5f,
		0.5f,0.5f,-0.5f,
		0.5f,0.5f, 0.5f,
		-0.5f,0.5f,-0.5f,
		-0.5f,0.5f, 0.5f
};

#define MAX_JOINTS 256

typedef struct {
	char name[64];
	int parent_index;
	mat4 bindPose;// aka world, aka localToWorld
    mat4 inverseBindPose;// aka inverseWorld, aka worldToLocal
    mat4 bindPoseLocal;
	vec3 bindLocalPosition;
	versor bindLocalRotation;
	vec3 bindLocalScale;
	vec3 localPosition;
	vec4 localRotation;
	vec3 localScale;
	mat4 animatedWorldTransform;
} Joint;

typedef struct Skeleton {
	Joint joints[MAX_JOINTS];
	int count;
	cgltf_data *cgltf_data;
    vec3 center_of_mass;
	vec3 pelvis_to_center_of_mass_offset;
} Skeleton;

typedef struct Human {
	cgltf_data* cgltf_data;
	Skeleton skeleton;
} Human;

typedef enum HumanJoint {
	HumanJoint_None,

	HumanJoint_Root,
	HumanJoint_Pelvis,
	HumanJoint_Spine01,
	HumanJoint_Spine02,
	HumanJoint_Spine03,
	HumanJoint_ClavicleL,
	HumanJoint_ClavicleR,
	HumanJoint_UpperArmL,
	HumanJoint_UpperArmR,
	HumanJoint_LowerArmL,
	HumanJoint_LowerArmR,
	HumanJoint_HandL,
	HumanJoint_HandR,
	HumanJoint_FingerIndexL1,
	HumanJoint_FingerIndexR1,
	HumanJoint_FingerMiddleL1,
	HumanJoint_FingerMiddleR1,
	HumanJoint_FingerRingL1,
	HumanJoint_FingerRingR1,
	HumanJoint_FingerPinkyL1,
	HumanJoint_FingerPinkyR1,
	HumanJoint_FingerThumbL1,
	HumanJoint_FingerThumbR1,

	HumanJoint_FingerIndexL2,
	HumanJoint_FingerIndexR2,
	HumanJoint_FingerMiddleL2,
	HumanJoint_FingerMiddleR2,
	HumanJoint_FingerRingL2,
	HumanJoint_FingerRingR2,
	HumanJoint_FingerPinkyL2,
	HumanJoint_FingerPinkyR2,
	HumanJoint_FingerThumbL2,
	HumanJoint_FingerThumbR2,

	HumanJoint_FingerIndexL3,
	HumanJoint_FingerIndexR3,
	HumanJoint_FingerMiddleL3,
	HumanJoint_FingerMiddleR3,
	HumanJoint_FingerRingL3,
	HumanJoint_FingerRingR3,
	HumanJoint_FingerPinkyL3,
	HumanJoint_FingerPinkyR3,
	HumanJoint_FingerThumbL3,
	HumanJoint_FingerThumbR3,

	HumanJoint_Neck,
	HumanJoint_Head,
	HumanJoint_ThighL,
	HumanJoint_ThighR,
	HumanJoint_CalfL,
	HumanJoint_CalfR,
	HumanJoint_FootL,
	HumanJoint_FootR,
	HumanJoint_FootBallL,
	HumanJoint_FootBallR,

	HumanJoint_End
} HumanJoint;

typedef struct {
	char sourceName[128];
	enum HumanJoint targetJoint;
} JointMapping;

typedef struct {
	JointMapping mappings[MAX_JOINTS];
	size_t count;
} HumanJointMap;

typedef struct HumanJointType {
	char name[128];
	HumanJoint joint;
} HumanJointType;

static const HumanJointType humanJointTable[] = {
		{"NONE", HumanJoint_None},
		{"Root", HumanJoint_Root},
		{"Pelvis", HumanJoint_Pelvis},
		{"Spine01", HumanJoint_Spine01},
		{"Spine02", HumanJoint_Spine02},
		{"Spine03", HumanJoint_Spine03},
		{"ClavicleL", HumanJoint_ClavicleL},
		{"ClavicleR", HumanJoint_ClavicleR},
		{"UpperArmL", HumanJoint_UpperArmL},
		{"UpperArmR", HumanJoint_UpperArmR},
		{"LowerArmL", HumanJoint_LowerArmL},
		{"LowerArmR", HumanJoint_LowerArmR},
		{"HandL", HumanJoint_HandL},
		{"HandR", HumanJoint_HandR},
		{"FingerIndexL1", HumanJoint_FingerIndexL1},
		{"FingerIndexR1", HumanJoint_FingerIndexR1},
		{"FingerMiddleL1", HumanJoint_FingerMiddleL1},
		{"FingerMiddleR1", HumanJoint_FingerMiddleR1},
		{"FingerRingL1", HumanJoint_FingerRingL1},
		{"FingerRingR1", HumanJoint_FingerRingR1},
		{"FingerPinkyL1", HumanJoint_FingerPinkyL1},
		{"FingerPinkyR1", HumanJoint_FingerPinkyR1},
		{"FingerThumbL1", HumanJoint_FingerThumbL1},
		{"FingerThumbR1", HumanJoint_FingerThumbR1},
		{"FingerIndexL2", HumanJoint_FingerIndexL2},
		{"FingerIndexR2", HumanJoint_FingerIndexR2},
		{"FingerMiddleL2", HumanJoint_FingerMiddleL2},
		{"FingerMiddleR2", HumanJoint_FingerMiddleR2},
		{"FingerRingL2", HumanJoint_FingerRingL2},
		{"FingerRingR2", HumanJoint_FingerRingR2},
		{"FingerPinkyL2", HumanJoint_FingerPinkyL2},
		{"FingerPinkyR2", HumanJoint_FingerPinkyR2},
		{"FingerThumbL2", HumanJoint_FingerThumbL2},
		{"FingerThumbR2", HumanJoint_FingerThumbR2},
		{"FingerIndexL3", HumanJoint_FingerIndexL3},
		{"FingerIndexR3", HumanJoint_FingerIndexR3},
		{"FingerMiddleL3", HumanJoint_FingerMiddleL3},
		{"FingerMiddleR3", HumanJoint_FingerMiddleR3},
		{"FingerRingL3", HumanJoint_FingerRingL3},
		{"FingerRingR3", HumanJoint_FingerRingR3},
		{"FingerPinkyL3", HumanJoint_FingerPinkyL3},
		{"FingerPinkyR3", HumanJoint_FingerPinkyR3},
		{"FingerThumbL3", HumanJoint_FingerThumbL3},
		{"FingerThumbR3", HumanJoint_FingerThumbR3},
		{"Neck", HumanJoint_Neck},
		{"Head", HumanJoint_Head},
		{"ThighL", HumanJoint_ThighL},
		{"ThighR", HumanJoint_ThighR},
		{"CalfL", HumanJoint_CalfL},
		{"CalfR", HumanJoint_CalfR},
		{"FootL", HumanJoint_FootL},
		{"FootR", HumanJoint_FootR},
		{"FootBallL", HumanJoint_FootBallL},
		{"FootBallR", HumanJoint_FootBallR}
};

static const size_t humanJointTableCount = sizeof(humanJointTable) / sizeof(humanJointTable[0]);

HumanJoint humanJointFromString(const char* name) {
	for (size_t i = 0; i < humanJointTableCount; i++) {
		if (strcasecmp(name, humanJointTable[i].name) == 0) {
			return humanJointTable[i].joint;
		}
	}

	fprintf(stderr, "Unknown HumanJoint name: %s\n", name);
	return HumanJoint_None;
}

const char* humanJointToString(HumanJoint joint) {
	for (size_t i = 0; i < humanJointTableCount; i++) {
		if (humanJointTable[i].joint == joint) {
			return humanJointTable[i].name;
		}
	}
	fprintf(stderr, "Unknown HumanJoint enum value: %d\n", joint);
	return "UNKNOWN_JOINT";
}

int skeleton_index_of(Skeleton *skeleton, const char *jointName) {
    for (int i = 0; i < skeleton->count; i++) {
        if (strcmp(jointName, skeleton->joints[i].name) == 0) {
            return i;
        }
    }
    return -1;
}

int skeleton_index_of_human_joint(Skeleton *skeleton, HumanJointMap *jointMap, HumanJoint j) {
    for (int i = 0; i < jointMap->count; i++) {
        JointMapping jointMapping = jointMap->mappings[i];
        if (jointMapping.targetJoint == j) {
            return skeleton_index_of(skeleton, jointMapping.sourceName);
        }
    }
    return -1;
}

int loadJointMappingFile(const char* filename, HumanJointMap* map) {
	FILE* f = fopen(filename, "r");
	if (!f) {
		perror("Failed to open mapping file");
		return -1;
	}

	char line[512];
	while (fgets(line, sizeof(line), f)) {
		// Trim whitespace
		char* start = line;
		while (isspace((unsigned char)*start)) start++;
		if (*start == '#' || *start == '\0') {
			// Comment or empty line
			continue;
		}

		char jointName[128];
		char sourceName[128];

		char* eq = strchr(start, '=');
		if (!eq) {
			fprintf(stderr, "Malformed line in %s (no '=' found): %s\n", filename, line);
			continue;
		}

		// split on =
		*eq = '\0'; // replace '=' with null terminator
		char* left = start;
		char* right = eq + 1;

		while (isspace((unsigned char)*left)) left++;
		while (*right && isspace((unsigned char)*right)) right++;

		char* endLeft = left + strlen(left) - 1;
		while (endLeft > left && isspace((unsigned char)*endLeft)) {
			*endLeft = '\0';
			endLeft--;
		}

		char* endRight = right + strlen(right) - 1;
		while (endRight > right && isspace((unsigned char)*endRight)) {
			*endRight = '\0';
			endRight--;
		}

		// left=humanJointName, right=sourceJointName
		strncpy(jointName, left, 127);
		jointName[127] = '\0';

		strncpy(sourceName, right, 127);
		sourceName[127] = '\0';

		enum HumanJoint j = humanJointFromString(jointName);
		if (j != HumanJoint_None) {
			//printf("joint %s=%s\n", jointName, sourceName);
			JointMapping jm;
			strcpy(jm.sourceName, sourceName);
			jm.targetJoint = j;
			map->mappings[map->count++] = jm;
		} else {
			fprintf(stderr, "Unknown joint name '%s' in %s\n", jointName, filename);
		}
	}

	fclose(f);
	return 0;
}

static void skeleton_set_local_transform_from_cgltf(Skeleton *skeleton, int jointIndex, cgltf_node *node) {
    // todo use cgltf_node_transform_world/cgltf_node_transform_local ?

    mat4 local;

    float t[3] = {0.0f,0.0f,0.0f};
    float r[4] = {0.0f,0.0f,0.0f,1.0f};
    float s[3] = {1.0f,1.0f,1.0f};

    if (node->has_translation) {
        t[0] = node->translation[0];
        t[1] = node->translation[1];
        t[2] = node->translation[2];
    }
    if (node->has_rotation) {
        r[0] = node->rotation[0];
        r[1] = node->rotation[1];
        r[2] = node->rotation[2];
        r[3] = node->rotation[3];
    }
    if (node->has_scale) {
        s[0] = node->scale[0];
        s[1] = node->scale[1];
        s[2] = node->scale[2];
    }

    glm_vec3_copy(t, skeleton->joints[jointIndex].bindLocalPosition);
    glm_vec4_copy(r, skeleton->joints[jointIndex].bindLocalRotation);
    glm_vec3_copy(s, skeleton->joints[jointIndex].bindLocalScale);

//    mat4 T, R, S;
//
//    glm_translate_make(T, (vec3){t[0], t[1], t[2]});
//    glm_quat_mat4((versor){r[0], r[1], r[2], r[3]}, R);
//    glm_scale_make(S, (vec3){s[0], s[1], s[2]});

//    glm_mat4_identity(local);
//    glm_mat4_mulN((mat4 *[]){&T, &R, &S}, 3, local);
//    glm_mat4_copy(local, skeleton->joints[jointIndex].bindPoseLocal);
}

static void skeleton_compute_bind_pose_joint(Skeleton *skeleton, int jointIndex) {
    int current = jointIndex;

    mat4 world_transform;
    glm_mat4_identity(world_transform);
    mat4 temp;
    while (current != -1) {
        mat4 local;
        glm_mat4_identity(local);

        mat4 T, R, S;

        glm_translate_make(T, skeleton->joints[current].bindLocalPosition);
        glm_quat_mat4(skeleton->joints[current].bindLocalRotation, R);
        glm_scale_make(S, skeleton->joints[current].bindLocalScale);

        glm_mat4_mulN((mat4 *[]){&T,&R,&S}, 3, local);
        glm_mat4_copy(local, skeleton->joints[jointIndex].bindPoseLocal);

        glm_mat4_mul(local, world_transform, temp);
        glm_mat4_copy(temp, world_transform);

        current = skeleton->joints[current].parent_index;
    }

    glm_mat4_copy(world_transform, skeleton->joints[jointIndex].bindPose);
    mat4 inverseWorld;// aka inverseBindPose, aka worldToLocal
    glm_mat4_inv(world_transform, inverseWorld);
    glm_mat4_copy(inverseWorld, skeleton->joints[jointIndex].inverseBindPose);
}

static void skeleton_compute_bind_pose(Skeleton *skeleton) {
	for (int i = 0; i < skeleton->count; i++) {
		skeleton_compute_bind_pose_joint(skeleton, i);
	}
}

//static void skeleton_compute_bind_pose(Skeleton *skeleton, int jointIndex, cgltf_node* node) {
//	// todo use cgltf_node_transform_world/cgltf_node_transform_local ?
//	cgltf_node* current = node;
//
//	mat4 world_transform;
//	glm_mat4_identity(world_transform);
//	mat4 temp;
//	while (current) {
//		mat4 local;
//
//		float t[3] = {0.0f,0.0f,0.0f};
//		float r[4] = {0.0f,0.0f,0.0f,1.0f};
//		float s[3] = {1.0f,1.0f,1.0f};
//
//		if (current->has_translation) {
//			t[0] = current->translation[0];
//			t[1] = current->translation[1];
//			t[2] = current->translation[2];
//		}
//		if (current->has_rotation) {
//			r[0] = current->rotation[0];
//			r[1] = current->rotation[1];
//			r[2] = current->rotation[2];
//			r[3] = current->rotation[3];
//		}
//		if (current->has_scale) {
//			s[0] = current->scale[0];
//			s[1] = current->scale[1];
//			s[2] = current->scale[2];
//		}
//
//		mat4 T, R, S;
//
//		glm_translate_make(T, (vec3){t[0], t[1], t[2]});
//		glm_quat_mat4((versor){r[0], r[1], r[2], r[3]}, R);
//		glm_scale_make(S, (vec3){s[0], s[1], s[2]});
//
//		glm_mat4_identity(local);
//		glm_mat4_mulN((mat4 *[]){&T, &R, &S}, 3, local);
//
//		glm_mat4_mul(local, world_transform, temp);
//		glm_mat4_copy(temp, world_transform);
//
//        if (current == node) {
//            glm_vec3_copy(t, skeleton->joints[jointIndex].bindLocalPosition);
//            glm_vec4_copy(r, skeleton->joints[jointIndex].bindLocalRotation);
//            glm_vec3_copy(s, skeleton->joints[jointIndex].bindLocalScale);
//            glm_mat4_copy(local, skeleton->joints[jointIndex].bindPoseLocal);
//        }
//
//		current = current->parent;
//	}
//
//	glm_mat4_copy(world_transform, skeleton->joints[jointIndex].bindPose);
//	mat4 inverseWorld;// aka inverseBindPose, aka worldToLocal
//	glm_mat4_inv(world_transform, inverseWorld);
//	glm_mat4_copy(inverseWorld, skeleton->joints[jointIndex].inverseBindPose);
//}

static GLuint compile_shader(GLenum type, const char* src) {
	GLuint s = glCreateShader(type);
	glShaderSource(s, 1, &src, NULL);
	glCompileShader(s);
	GLint success;
	glGetShaderiv(s, GL_COMPILE_STATUS, &success);
	if(!success) {
		char log[512];
		glGetShaderInfoLog(s, 512, NULL, log);
		fprintf(stderr, "Shader compilation error: %s\n", log);
		exit(1);
	}
	return s;
}

static GLuint create_program(char *src_vs, char *src_fs) {
	GLuint vs = compile_shader(GL_VERTEX_SHADER, src_vs);
	GLuint fs = compile_shader(GL_FRAGMENT_SHADER, src_fs);

	GLuint prog = glCreateProgram();
	glAttachShader(prog, vs);
	glAttachShader(prog, fs);
	glLinkProgram(prog);

	glDeleteShader(vs);
	glDeleteShader(fs);

	GLint linked;
	glGetProgramiv(prog, GL_LINK_STATUS, &linked);
	if(!linked) {
		char log[512];
		glGetProgramInfoLog(prog, 512, NULL, log);
		fprintf(stderr, "Program link error: %s\n", log);
	}
	return prog;
}

void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "Error: %d, %s\n", error, description);
}

void draw_grid() {
	for (int x = -5; x < 5 + 1; x++) {
		draw_line((vec3){ x, 0.0f, -5.0f}, (vec3){x, 0.0f, 5.0f}, (vec3){0.3f, 0.3f, 0.3f});
	}
	for (int z = -5; z < 5 + 1; z++) {
		draw_line((vec3){ -5.0f, 0.0f, z}, (vec3){5.0f, 0.0f, z}, (vec3){0.3f, 0.3f, 0.3f});
	}
}

void draw_skeleton_bindpose(Skeleton *skeleton, vec3 offset) {
    for (size_t j = 0; j < skeleton->count; j++) {
        int parent = skeleton->joints[j].parent_index;
        if (parent == -1) continue;

        bool pelvis = skeleton->joints[parent].parent_index == -1;
        if (pelvis) continue;

        vec3 parentPos = {0};
        vec3 pos = {0};
        glm_mat4_mulv3(skeleton->joints[parent].bindPose, parentPos, 1.0f, parentPos);
        glm_mat4_mulv3(skeleton->joints[j].bindPose, pos, 1.0f, pos);

        vec3 a = {0};
        glm_vec3_add(parentPos, offset, a);
        vec3 b = {0};
        glm_vec3_add(pos, offset, b);
        draw_line(a, b, (vec3) { 0.0f, 1.0f, 0.0f });
    }
}

void draw_skeleton_bindpose_dashed(Skeleton *skeleton, vec3 offset) {
	for (size_t j = 0; j < skeleton->count; j++) {
		int parent = skeleton->joints[j].parent_index;
        if (parent == -1) continue;

        bool pelvis = skeleton->joints[parent].parent_index == -1;
        if (pelvis) continue;

        vec3 parentPos = {0};
        vec3 pos = {0};
        glm_mat4_mulv3(skeleton->joints[parent].bindPose, parentPos, 1.0f, parentPos);
        glm_mat4_mulv3(skeleton->joints[j].bindPose, pos, 1.0f, pos);

        vec3 a = {0};
        glm_vec3_add(parentPos, offset, a);
        vec3 b = {0};
        glm_vec3_add(pos, offset, b);
        draw_line_dashed(a, b, (vec3){1.0f, 1.0f, 0.0f}, 0.01f, 0.03f);
    }
}

void skeleton_estimate_center_of_mass(Skeleton *skeleton, HumanJointMap *jointMap, vec3 out_center_of_mass, vec3 out_pelvis_to_center_of_mass_offset) {
	vec3 pelvis = {0};
	vec3 shoulder_r = {0};
    vec3 shoulder_l = {0};
    vec3 foot_r = {0};
    vec3 foot_l = {0};
	glm_mat4_mulv3(skeleton->joints[skeleton_index_of_human_joint(skeleton, jointMap, HumanJoint_Pelvis)].bindPose, pelvis, 1.0f, pelvis);
	glm_mat4_mulv3(skeleton->joints[skeleton_index_of_human_joint(skeleton, jointMap, HumanJoint_ClavicleR)].bindPose, shoulder_r, 1.0f, shoulder_r);
    glm_mat4_mulv3(skeleton->joints[skeleton_index_of_human_joint(skeleton, jointMap, HumanJoint_ClavicleL)].bindPose, shoulder_l, 1.0f, shoulder_l);
    glm_mat4_mulv3(skeleton->joints[skeleton_index_of_human_joint(skeleton, jointMap, HumanJoint_FootR)].bindPose, foot_r, 1.0f, foot_r);
    glm_mat4_mulv3(skeleton->joints[skeleton_index_of_human_joint(skeleton, jointMap, HumanJoint_FootL)].bindPose, foot_l, 1.0f, foot_l);

    // crude center of mass estimation:
    // roughly halfway between the feet (actually the floor: assume feet are on the ground and don't use foot bone Y as they can differ)
    // plus a fudge factor to land roughly around the belly button area
    float avg_clavicle_y = (shoulder_r[1] + shoulder_l[1]) / 2.0f;
    float avg_y = avg_clavicle_y / 2.0f;

    avg_y *= 1.45f; // the fudge factor

    float avg_z = (pelvis[2] + shoulder_r[2] + shoulder_l[2] + foot_r[2] + foot_l[2]) / 5.0f;

	out_center_of_mass[0] = 0.0;
	out_center_of_mass[1] = avg_y;
	out_center_of_mass[2] = avg_z;

	// calc offset from pelvis to center of mass
	glm_vec3_sub(out_center_of_mass, pelvis, out_pelvis_to_center_of_mass_offset);
}

void draw_skeleton_center_of_mass(Skeleton *skeleton, HumanJointMap *jointMap, vec3 offset) {
    vec3 v = {0};
    glm_vec3_copy(skeleton->center_of_mass, v);
    glm_vec3_add(offset, v, v);
    draw_sphere(v, 0.05, 4, 4, (vec3){0.25, 0.25, 1.0f});
}

void draw_skeleton_animated(Skeleton *skeleton, vec3 offset) {
	for (size_t j = 0; j < skeleton->count; j++) {
		int parent = skeleton->joints[j].parent_index;

		// draw joint origin (sphere)
		vec3 color = { 0.0f, 1.0f, 0.0f };
		if (parent == -1) {
			color[0] = 0.25f;
			color[1] = 0.25f;
			color[2] = 1.0f;
		}
		vec3 pos = {0};
		glm_mat4_mulv3(skeleton->joints[j].animatedWorldTransform, pos, 1.0f, pos);
		glm_vec3_add(pos, offset, pos);
		draw_sphere(pos, 0.025f, 4, 4, color);

		if (parent == -1) continue;

        bool pelvis = skeleton->joints[parent].parent_index == -1;
        if (pelvis) continue;

		// draw line from parent to child
        vec3 parentPos = {0};
        glm_mat4_mulv3(skeleton->joints[parent].animatedWorldTransform, parentPos, 1.0f, parentPos);
        glm_vec3_add(parentPos, offset, parentPos);

        draw_line(pos, parentPos, (vec3) { 0.0f, 1.0f, 0.0f });
	}
}

int load_skeleton(Skeleton *skeleton, const char *gltfFileName) {
	cgltf_options options = {0};
	cgltf_data* data = NULL;
	cgltf_result result = cgltf_parse_file(&options, gltfFileName, &data);
	if (result != cgltf_result_success) {
		fprintf(stderr, "Failed to parse gltf file ('%s')\n", gltfFileName);
		return -1;
	}
	skeleton->cgltf_data = data;

	result = cgltf_load_buffers(&options, data, gltfFileName);
	if (result != cgltf_result_success) {
		fprintf(stderr, "Failed to load gltf buffers.\n");
		cgltf_free(data);
		return -1;
	}

	if (data->skins_count == 0) {
		fprintf(stderr, "No skins found in the GLTF.\n");
		cgltf_free(data);
		return -1;
	}

	if (data->skins_count > 1) {
		fprintf(stderr, "Too many skins found (%d).\n", data->skins_count);
		cgltf_free(data);
		return -1;
	}

	cgltf_skin* skin = &data->skins[0];

	size_t joint_count = skin->joints_count;
	if (joint_count == 0) {
		fprintf(stderr, "Skin has no joints.\n");
		return -1;
	}
	skeleton->count = joint_count;

	// Map joints to indices for easy lookup
	// (skin->joints[j] is a node pointer)
	// We'll assume the order given by the skin is what we use.
	for (size_t j = 0; j < joint_count; j++) {
		cgltf_node* joint_node = skin->joints[j];
		const char* name = joint_node->name ? joint_node->name : "unnamed";
		strncpy(skeleton->joints[j].name, name, 63);
		skeleton->joints[j].name[63] = '\0';

		// find parent in the joint list
		// If node->parent isn't in the joint list, parent_index = -1 (root)
		if (joint_node->parent) {
			int parentIndex = -1;
			for (size_t p = 0; p < joint_count; p++) {
				if (skin->joints[p] == joint_node->parent) {
					parentIndex = (int)p;
					break;
				}
			}
			skeleton->joints[j].parent_index = parentIndex;
		} else {
			skeleton->joints[j].parent_index = -1;
		}
	}

    // set joint transforms from cgltf nodes
    for (size_t j = 0; j < joint_count; j++) {
        skeleton_set_local_transform_from_cgltf(skeleton, j, skin->joints[j]);
    }

    // compute bindpose / inverse bindpose
	skeleton_compute_bind_pose(skeleton);
	return 0;
}

void create_line_buffer(LineBuffer *lineBuffer) {
	lineBuffer->line_prog = create_program(line_vertex_shader_src, line_fragment_shader_src);
	lineBuffer->line_mvp_loc = glGetUniformLocation(lineBuffer->line_prog, "MVP");

	glGenVertexArrays(1, &lineBuffer->line_vao);
	glGenBuffers(1, &lineBuffer->line_vbo);

	glBindVertexArray(lineBuffer->line_vao);
	glBindBuffer(GL_ARRAY_BUFFER, lineBuffer->line_vbo);

	// position (layout=0)
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);

	// color (layout=1)
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

	glBindVertexArray(0);

	lineBuffer->capacity = 1024; // capacity in floats, 12 per line if storing 2 vertices per line
	lineBuffer->vertices = (float*)malloc(sizeof(float) * lineBuffer->capacity);
	lineBuffer->count = 0;
}


void draw_line_buffer(LineBuffer *lineBuffer, mat4 mvp) {
	glUseProgram(lineBuffer->line_prog);
	glUniformMatrix4fv(lineBuffer->line_mvp_loc, 1, GL_FALSE, (float*)mvp);

	glBindVertexArray(lineBuffer->line_vao);
	glBindBuffer(GL_ARRAY_BUFFER, lineBuffer->line_vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)*lineBuffer->count, lineBuffer->vertices, GL_DYNAMIC_DRAW);

	glDrawArrays(GL_LINES, 0, (GLsizei)(lineBuffer->count / 6));

	lineBuffer->count = 0; // reset for next frame if you want

	glBindVertexArray(0);
	glUseProgram(0);
}


void delete_line_buffer(LineBuffer *lineBuffer) {
	free(lineBuffer->vertices);
	glDeleteBuffers(1, &lineBuffer->line_vbo);
	glDeleteVertexArrays(1, &lineBuffer->line_vao);
	glDeleteProgram(lineBuffer->line_prog);
}

void animation_compute_final_transforms_joint(Skeleton *skeleton, int jointIndex, mat4 parent) {
	mat4 T, R, S;
	glm_translate_make(T, skeleton->joints[jointIndex].localPosition);
	glm_quat_mat4(skeleton->joints[jointIndex].localRotation, R);
	glm_scale_make(S, skeleton->joints[jointIndex].localScale);

	mat4 local;
	glm_mat4_identity(local);
	glm_mat4_mulN((mat4 *[]){&T, &R, &S}, 3, local);

	mat4 world;
	glm_mat4_mul(parent, local, world);
	glm_mat4_copy(world, skeleton->joints[jointIndex].animatedWorldTransform);

	for (int i = 0; i < skeleton->count; i++) {
		if (skeleton->joints[i].parent_index == jointIndex) {
			animation_compute_final_transforms_joint(skeleton, i, world);
		}
	}
}

void animation_compute_final_transforms(Skeleton *skeleton) {
    mat4 identity;
    glm_mat4_identity(identity);

    animation_compute_final_transforms_joint(skeleton, 0, identity);
}

void skeleton_reset_to_bind_pose(Skeleton *skeleton) {
	for (int i = 0; i < skeleton->count; i++) {
		// todo maybe shouldnt be here
		glm_mat4_copy(skeleton->joints[i].bindPose, skeleton->joints[i].animatedWorldTransform);

		glm_vec3_copy(skeleton->joints[i].bindLocalPosition, skeleton->joints[i].localPosition);
		glm_vec4_copy(skeleton->joints[i].bindLocalRotation, skeleton->joints[i].localRotation);
		glm_vec3_copy(skeleton->joints[i].bindLocalScale, skeleton->joints[i].localScale);
	}
}

typedef struct AnimationFrame {
	vec3 translation;
	versor rotation;
	vec3 scale;
} AnimationFrame;

typedef struct AnimationChannel {
	char name[128];
	int frame_count;
	cgltf_animation_path_type path_type;
	AnimationFrame *frames;
	float *times;
} AnimationChannel;

typedef struct Animation {
	AnimationChannel *channels;
	int num_channels;
} Animation;

void load_anim(Skeleton *skeleton, Animation *out, Arena *arena) {
	cgltf_animation anim = skeleton->cgltf_data->animations[0];

	out->num_channels = anim.channels_count;
	out->channels = arena_alloc_zeroed(arena, sizeof(AnimationChannel) * anim.channels_count);
	for (int i = 0; i < anim.channels_count; i++) {
		cgltf_animation_channel channel = anim.channels[i];
		AnimationChannel *outChannel = &out->channels[i];
		outChannel->path_type = channel.target_path;
		char *sourceJointName = channel.target_node->name;
		strncpy(outChannel->name, sourceJointName, 127);
		outChannel->name[127] = '\0';

		cgltf_animation_sampler *sampler = channel.sampler;
		cgltf_accessor *inputAccessor = sampler->input;
		cgltf_accessor *outputAccessor = sampler->output;
		size_t expectedCount = inputAccessor->count;
		if (inputAccessor->component_type != cgltf_component_type_r_32f) {
			fprintf(stderr, "unexpected component type %i", inputAccessor->component_type);
			exit(1);
		}

		if (inputAccessor->count != outputAccessor->count) {
			fprintf(stderr, "input %d output %d mismatch", inputAccessor->count, outputAccessor->count);
			exit(1);
		}

		size_t numInputFloats = cgltf_accessor_unpack_floats(inputAccessor, NULL, 0);
		float *inBuf = malloc(sizeof(float) * numInputFloats);
		size_t readResult = cgltf_accessor_unpack_floats(inputAccessor, inBuf, numInputFloats);
		if (readResult / cgltf_num_components(inputAccessor->type) != expectedCount) {
			fprintf(stderr, "Error reading input accessor: expected %d/%d elements got %d (allocated %d floats)\n", expectedCount, inputAccessor->count, readResult, numInputFloats);
			exit(1);
		}

		size_t numOutputFloats = cgltf_accessor_unpack_floats(outputAccessor, NULL, 0);
		float *outBuf = malloc(sizeof(float) * numOutputFloats);
		readResult = cgltf_accessor_unpack_floats(outputAccessor, outBuf, numOutputFloats);
		if (readResult / cgltf_num_components(outputAccessor->type) != expectedCount) {
			fprintf(stderr, "Error reading output accessor: expected %d/%d elements got %d (allocated %d floats)\n", expectedCount, outputAccessor->count, readResult, numOutputFloats);
			exit(1);
		}

		outChannel->frame_count = numInputFloats;
		outChannel->times = arena_alloc_zeroed(arena, sizeof(float) * numInputFloats);
		memcpy(outChannel->times, inBuf, sizeof(float) * numInputFloats);
		outChannel->frames = arena_alloc_zeroed(arena, sizeof(AnimationFrame) * numInputFloats);
		for (int j = 0; j < numInputFloats; j++) {
			AnimationFrame *outFrame = &outChannel->frames[j];

			if (channel.target_path == cgltf_animation_path_type_translation) {
				vec3 translation = {
						outBuf[(j * 3) + 0],
						outBuf[(j * 3) + 1],
						outBuf[(j * 3) + 2]
				};
				glm_vec3_copy(translation, outFrame->translation);
			} else if (channel.target_path == cgltf_animation_path_type_rotation) {
				versor rotation = {
						outBuf[(j * 4) + 0],
						outBuf[(j * 4) + 1],
						outBuf[(j * 4) + 2],
						outBuf[(j * 4) + 3]
				};
				glm_quat_normalize(rotation);
				glm_quat_copy(rotation, outFrame->rotation);
			} else if (channel.target_path == cgltf_animation_path_type_scale) {
				vec3 scale = {
						outBuf[(j * 3) + 0],
						outBuf[(j * 3) + 1],
						outBuf[(j * 3) + 2]
				};
				glm_vec3_copy(scale, outFrame->scale);
			}
		}

		free(inBuf);
		free(outBuf);
	}
}

void animation_sample(Skeleton *skeleton, Animation *anim, float t) {
	for (int i = 0; i < anim->num_channels; i++) {
		AnimationChannel *channel = &anim->channels[i];
		char *sourceJointName = channel->name;

		// find the joint index
		int jointIndex = skeleton_index_of(skeleton, sourceJointName);
		if (jointIndex == -1) {
			fprintf(stderr, "animation_sample cannot find joint %s\n", sourceJointName);
			continue;
		}

		// default to frame 0
		int frameIndex = 0;

		// loop time
		t = fmodf(t, (channel->times[channel->frame_count - 1]));

		// find frame at time t
		for (int j = 0; j < channel->frame_count; j++) {
			float time = channel->times[j];
			if (time >= t) {
				frameIndex = glm_max(0, j - 1);
				break;
			}
		}

		if (channel->path_type == cgltf_animation_path_type_translation) {
			glm_vec3_copy(channel->frames[frameIndex].translation, skeleton->joints[jointIndex].localPosition);
		} else if (channel->path_type == cgltf_animation_path_type_rotation) {
			glm_quat_copy(channel->frames[frameIndex].rotation, skeleton->joints[jointIndex].localRotation);
		} else if (channel->path_type == cgltf_animation_path_type_scale) {
			glm_vec3_copy(channel->frames[frameIndex].scale, skeleton->joints[jointIndex].localScale);
		}
	}
}

void load_and_dump() {
	cgltf_options options = {0};
	cgltf_data* data = NULL;
	char *gltfFileName = "target.gltf";
	cgltf_result result = cgltf_parse_file(&options, gltfFileName, &data);
	if (result != cgltf_result_success) {
		fprintf(stderr, "Failed to parse gltf file ('%s')\n", gltfFileName);
        exit(1);
	}

	result = cgltf_load_buffers(&options, data, gltfFileName);
	if (result != cgltf_result_success) {
		fprintf(stderr, "Failed to load gltf buffers.\n");
		cgltf_free(data);
        exit(1);
	}

	cgltf_data out;

	cgltf_validate(data);
	if (cgltf_validate(&out) != cgltf_result_success) {
		fprintf(stderr, "fail");
		exit(1);
	}

//	cgltf_options gltf_write_options = {0};
//	//cgltf_data* data = /* TODO must be valid data */;
//	cgltf_result gltf_write_result = cgltf_write_file(&gltf_write_options, "dump.gltf", data);
//	if (gltf_write_result != cgltf_result_success) {
//		fprintf(stderr, "Failed to write output file\n");
//		return -1;
//	}
}

void skeleton_get_worldspace_dir_to_joint(Skeleton *sourceSkeleton, HumanJointMap *sourceJointMap, HumanJoint j1, HumanJoint j2, vec3 dest) {
	int srcIndexJ1 = skeleton_index_of_human_joint(sourceSkeleton, sourceJointMap, j1);
	int srcIndexJ2 = skeleton_index_of_human_joint(sourceSkeleton, sourceJointMap, j2);
	if (srcIndexJ1 == -1) {
		fprintf(stderr, "Error: Joint %d not found in source skeleton.\n", j1);
		return;
	}
	if (srcIndexJ2 == -1) {
		fprintf(stderr, "Error: Joint %d not found in source skeleton.\n", j2);
		return;
	}
	int srcIndexParent = sourceSkeleton->joints[srcIndexJ1].parent_index;
	if (srcIndexParent == -1 || srcIndexParent >= sourceSkeleton->count) {
		fprintf(stderr, "Error: Invalid parent index %d for joint %d in source skeleton.\n", srcIndexParent, j1);
		return;
	}

	int indexJ1 = skeleton_index_of_human_joint(sourceSkeleton, sourceJointMap, j1);
	int indexJ2 = skeleton_index_of_human_joint(sourceSkeleton, sourceJointMap, j2);
	if (indexJ1 == -1) {
		fprintf(stderr, "Error: Joint %d not found in target skeleton.\n", j1);
		return;
	}
	if (indexJ2 == -1) {
		fprintf(stderr, "Error: Joint %d not found in target skeleton.\n", j2);
		return;
	}

	int indexParent = sourceSkeleton->joints[indexJ1].parent_index;
	if (indexParent == -1 || indexParent >= sourceSkeleton->count) {
		fprintf(stderr, "Error: Invalid parent index %d for joint %d.\n", indexParent, j1);
		return;
	}

	// joint positions in world space
	vec3 srcPositionJ1 = {0};
	vec3 srcPositionJ2 = {0};
	glm_mat4_mulv3(sourceSkeleton->joints[srcIndexJ1].bindPose, (vec3) {0, 0, 0}, 1.0f, srcPositionJ1);
	glm_mat4_mulv3(sourceSkeleton->joints[srcIndexJ2].bindPose, (vec3) {0, 0, 0}, 1.0f, srcPositionJ2);

	// find normalized dir from j1 to j2
	glm_vec3_sub(srcPositionJ2, srcPositionJ1, dest);
	glm_vec3_normalize(dest);
}

int skeleton_joint_get_child_count(Skeleton *skeleton, int jointIndex) {
	int children = 0;
	for (int i = 0; i < skeleton->count; i++) {
		if (skeleton->joints[i].parent_index == jointIndex) {
			children++;
		}
	}
	return 0;
}

void pose_match_joint(Skeleton *sourceSkeleton, HumanJointMap *sourceJointMap, Skeleton *skeleton, HumanJointMap *jointMap, HumanJoint j1, HumanJoint j2) {
	int srcIndexJ1 = skeleton_index_of_human_joint(sourceSkeleton, sourceJointMap, j1);
	int srcIndexJ2 = skeleton_index_of_human_joint(sourceSkeleton, sourceJointMap, j2);
	if (srcIndexJ1 == -1) {
		fprintf(stderr, "Error: Joint %d not found in source skeleton.\n", j1);
		return;
	}
	if (srcIndexJ2 == -1) {
		fprintf(stderr, "Error: Joint %d not found in source skeleton.\n", j2);
		return;
	}
	int srcIndexParent = sourceSkeleton->joints[srcIndexJ1].parent_index;
	if (srcIndexParent == -1 || srcIndexParent >= sourceSkeleton->count) {
		fprintf(stderr, "Error: Invalid parent index %d for joint %d in source skeleton.\n", srcIndexParent, j1);
		return;
	}

	int indexJ1 = skeleton_index_of_human_joint(skeleton, jointMap, j1);
	int indexJ2 = skeleton_index_of_human_joint(skeleton, jointMap, j2);
	if (indexJ1 == -1) {
		fprintf(stderr, "Error: Joint %d not found in target skeleton.\n", j1);
		return;
	}
	if (indexJ2 == -1) {
		fprintf(stderr, "Error: Joint %d not found in targetskeleton.\n", j2);
		return;
	}

	int indexParent = skeleton->joints[indexJ1].parent_index;
	if (indexParent == -1 || indexParent >= skeleton->count) {
		fprintf(stderr, "Error: Invalid parent index %d for joint %d.\n", indexParent, j1);
		return;
	}

	// joint positions in world space
	vec3 srcPositionJ1 = {0};
	vec3 srcPositionJ2 = {0};
	glm_mat4_mulv3(sourceSkeleton->joints[srcIndexJ1].bindPose, (vec3){0, 0, 0}, 1.0f, srcPositionJ1);
	glm_mat4_mulv3(sourceSkeleton->joints[srcIndexJ2].bindPose, (vec3){0, 0, 0}, 1.0f, srcPositionJ2);

	vec3 positionJ1 = {0};
	vec3 positionJ2 = {0};
	glm_mat4_mulv3(skeleton->joints[indexJ1].bindPose, (vec3){0, 0, 0}, 1.0f, positionJ1);
	glm_mat4_mulv3(skeleton->joints[indexJ2].bindPose, (vec3){0, 0, 0}, 1.0f, positionJ2);

	// find normalized dir from j1 to j2
	vec3 srcDirJ1toJ2;
	glm_vec3_sub(srcPositionJ2, srcPositionJ1, srcDirJ1toJ2);
	glm_vec3_normalize(srcDirJ1toJ2);

	vec3 dirJ1toJ2;
	glm_vec3_sub(positionJ2, positionJ1, dirJ1toJ2);
	glm_vec3_normalize(dirJ1toJ2);

	// compute the difference from dirJ1toJ2 to dir
	versor delta;
	glm_quat_from_vecs(dirJ1toJ2, srcDirJ1toJ2, delta);
	glm_quat_normalize(delta);

	vec3 o;
	glm_quat_rotatev(delta, dirJ1toJ2, o);

	versor originalWorldRotationJ1;
	glm_mat4_quat(skeleton->joints[indexJ1].bindPose, originalWorldRotationJ1);

	versor newWorldRotationJ1;
	glm_quat_mul(delta, originalWorldRotationJ1, newWorldRotationJ1);
	glm_quat_normalize(newWorldRotationJ1);

	// convert to joint parent local space
	mat4 *parentWorldToLocalJ1 = &skeleton->joints[indexParent].inverseBindPose;
	versor parentR;
	glm_mat4_quat(*parentWorldToLocalJ1, parentR);

	versor newLocalRotationJ1;
	glm_quat_mul(parentR, newWorldRotationJ1, newLocalRotationJ1);
	glm_quat_norm(newLocalRotationJ1);

	glm_quat_copy(newLocalRotationJ1, skeleton->joints[indexJ1].bindLocalRotation);
}

void tpose_bone_align(Skeleton *skeleton, HumanJointMap *jointMap, HumanJoint j1, HumanJoint j2, vec3 dir) {
    int indexJ1 = skeleton_index_of_human_joint(skeleton, jointMap, j1);
    int indexJ2 = skeleton_index_of_human_joint(skeleton, jointMap, j2);
    if (indexJ1 == -1) {
        fprintf(stderr, "Error: Joint %d not found in skeleton.\n", j1);
        return;
    }
    if (indexJ2 == -1) {
        fprintf(stderr, "Error: Joint %d not found in skeleton.\n", j2);
        return;
    }

    int indexParent = skeleton->joints[indexJ1].parent_index;
    if (indexParent == -1 || indexParent >= skeleton->count) {
        fprintf(stderr, "Error: Invalid parent index %d for joint %d.\n", indexParent, j1);
        return;
    }

    // joint positions in world space
    vec3 positionJ1 = {0};
    vec3 positionJ2 = {0};
    glm_mat4_mulv3(skeleton->joints[indexJ1].bindPose, (vec3){0, 0, 0}, 1.0f, positionJ1);
    glm_mat4_mulv3(skeleton->joints[indexJ2].bindPose, (vec3){0, 0, 0}, 1.0f, positionJ2);

    // find normalized dir from j1 to j2
    vec3 dirJ1toJ2;
    glm_vec3_sub(positionJ2, positionJ1, dirJ1toJ2);
    glm_vec3_normalize(dirJ1toJ2);

    // compute the difference from dirJ1toJ2 to dir
    versor delta;
    glm_quat_from_vecs(dirJ1toJ2, dir, delta);
    glm_quat_normalize(delta);

    vec3 o;
    glm_quat_rotatev(delta, dirJ1toJ2, o);

    versor originalWorldRotationJ1;
    glm_mat4_quat(skeleton->joints[indexJ1].bindPose, originalWorldRotationJ1);

    versor newWorldRotationJ1;
    glm_quat_mul(delta, originalWorldRotationJ1, newWorldRotationJ1);
    glm_quat_normalize(newWorldRotationJ1);

    // convert to joint parent local space
    mat4 *parentWorldToLocalJ1 = &skeleton->joints[indexParent].inverseBindPose;
    versor parentR;
    glm_mat4_quat(*parentWorldToLocalJ1, parentR);

    versor newLocalRotationJ1;
    glm_quat_mul(parentR, newWorldRotationJ1, newLocalRotationJ1);
    glm_quat_norm(newLocalRotationJ1);

    glm_quat_copy(newLocalRotationJ1, skeleton->joints[indexJ1].bindLocalRotation);
}

#include <math.h>
#include <float.h>
#include "cglm/cglm.h" // for glm_* etc.

/* https://theorangeduck.com/page/simple-two-joint */
void two_bone_ik(
		const vec3 root,
		const vec3 joint,
		const vec3 end,
		const vec3 target,
		const vec3 pole,
		versor outBoneARot,
		versor outBoneBRot) {
	// 1. Lengths of the two bones
	float L1 = glm_vec3_distance(root, joint);
	float L2 = glm_vec3_distance(joint, end);

	// 2. 'forward' in bind pose (root->end)
	vec3 forward;
	glm_vec3_sub(end, root, forward);

	// 3. 'targetForward' from root->target
	vec3 targetForward;
	glm_vec3_sub(target, root, targetForward);
	float distTarget = glm_vec3_norm(targetForward);

	// Clamp distance to not exceed max extension
	float maxReach = L1 + L2;
	if (distTarget > maxReach) distTarget = maxReach;
	glm_vec3_normalize_to(targetForward, targetForward);
	glm_vec3_scale(targetForward, distTarget, targetForward);

	// 4. Construct the transform (basis) that aligns 'forward' -> X-axis
	//    and places 'pole' into that plane so we can do a 2D IK solve.
	//    We'll build a quaternion that rotates 'forward' onto +X,
	//    and also tries to push 'pole' so it's in -Z or +Z, etc.
	//    Then we rotate targetForward by that same quaternion => 2D.

	// We'll define:
	//   x-axis = forward direction
	//   up-axis (some Y) = cross(poleNormal, x)  [or depends on desired orientation]
	//   z-axis = cross(x, up)
	// (The article uses a slightly simpler approach, but we'll do it systematically.)

	// (a) If forward is near zero length, bail out:
	float fwdLen = glm_vec3_norm(forward);
	if (fwdLen < 1e-6f) {
		glm_quat_identity(outBoneARot);
		glm_quat_identity(outBoneBRot);
		return;
	}
	// (b) Normalize
	vec3 fwdNorm;
	glm_vec3_copy(forward, fwdNorm);
	glm_vec3_normalize(fwdNorm);

	// (c) Orthonormal basis: we want to push the 'pole' to define an up or binormal.
	//     Let's define the plane normal = cross(fwdNorm, pole). (If near zero => fallback.)
	vec3 planeNormal;
	glm_vec3_cross(fwdNorm, pole, planeNormal);
	if (glm_vec3_norm(planeNormal) < 1e-6f) {
		// fallback: if pole is collinear with forward, pick a default normal
		glm_vec3_copy((vec3){0,1,0}, planeNormal);
		glm_vec3_cross(fwdNorm, planeNormal, planeNormal);
	}
	glm_vec3_normalize(planeNormal);

	// upAxis = cross(planeNormal, fwdNorm)
	// so we have: x=fwdNorm, y=upAxis, z=planeNormal
	vec3 upAxis, rightAxis;
	glm_vec3_cross(planeNormal, fwdNorm, upAxis);
	// planeNormal is effectively "z," upAxis is "y," and fwdNorm is "x" (like an X-right-handed basis).
	// We'll rename for clarity:
	glm_vec3_copy(fwdNorm,   rightAxis);   // X
	vec3   axisZ; glm_vec3_copy(planeNormal, axisZ); // Z
	vec3   axisY; glm_vec3_copy(upAxis,      axisY); // Y

	// Build a rotation matrix that maps: (X->rightAxis, Y->axisY, Z->axisZ).
	mat4 basis;
	glm_mat4_identity(basis);
	// column 0 = rightAxis
	basis[0][0] = rightAxis[0];
	basis[1][0] = rightAxis[1];
	basis[2][0] = rightAxis[2];
	// column 1 = axisY
	basis[0][1] = axisY[0];
	basis[1][1] = axisY[1];
	basis[2][1] = axisY[2];
	// column 2 = axisZ
	basis[0][2] = axisZ[0];
	basis[1][2] = axisZ[1];
	basis[2][2] = axisZ[2];

	// We want a quaternion that rotates the bind forward => +X,
	// so effectively: q = rotation that transforms 'basis' into identity.
	// One approach is to take the transpose (since basis is orthonormal):
	mat4 invBasis;
	glm_mat4_transpose_to(basis, invBasis);

	versor qBasis;
	glm_mat4_quat(invBasis, qBasis);

	// 5. Rotate the original forward -> 2D, and also rotate 'targetForward' -> 2D
	vec3 fwd2d, tgt2d;
	glm_quat_rotatev(qBasis, forward,        fwd2d);
	glm_quat_rotatev(qBasis, targetForward,  tgt2d);

	// Now, in qBasis-space, the “forward” is purely along +X,
	// and the “target” is somewhere in the X-Y plane, with Z near zero.

	// 6. Solve 2D IK in that space.  (We just do the angle computations.)
	//
	// Let d = |tgt2d|; clamp to [0..L1+L2]
	float d = sqrtf(tgt2d[0]*tgt2d[0] + tgt2d[1]*tgt2d[1]);
	// Already clamped above, but just in case:
	if (d < 1e-6f) d = 1e-6f;

	// angleA = angle at bone A
	// cos(angleA) = (d^2 + L1^2 - L2^2) / (2 d L1)
	float cosA = (d*d + L1*L1 - L2*L2) / (2.0f * d * L1);
	if (cosA < -1.0f) cosA = -1.0f;
	if (cosA >  1.0f) cosA =  1.0f;
	float angleA = acosf(cosA);

	// The direction from +X to (tgt2d.x, tgt2d.y) is just atan2(y,x)
	float baseAngle = atan2f(tgt2d[1], tgt2d[0]);
	float shoulderAngle2D = baseAngle + angleA;

	// angleB = how much bone B bends
	// cosB = (L1^2 + L2^2 - d^2) / (2 L1 L2)
	float cosB = (L1*L1 + L2*L2 - d*d) / (2.0f * L1 * L2);
	if (cosB < -1.0f) cosB = -1.0f;
	if (cosB >  1.0f) cosB =  1.0f;
	float angleB = acosf(cosB);

	// Some prefer elbowAngle = π - angleB so that 0 => straight arm, π => folded
	float elbowAngle2D = M_PI - angleB;

	// 7. Build quaternions for these two angles *in the 2D plane* (around Z).
	//    Then rotate them back out of qBasis space.

	versor qShoulderLocal, qElbowLocal;
	glm_quat_identity(qShoulderLocal);
	glm_quat_identity(qElbowLocal);

	// Shoulder rotates around Z by shoulderAngle2D
	glm_quat(qShoulderLocal, 0.0f, 0.0f, sinf(shoulderAngle2D/2.0f), cosf(shoulderAngle2D/2.0f));

	// Elbow rotates around Z by elbowAngle2D
	glm_quat(qElbowLocal, 0.0f, 0.0f, sinf(elbowAngle2D/2.0f), cosf(elbowAngle2D/2.0f));

	// 8. “Undo” the qBasis rotation => we get final orientation in world space.
	// outBoneARot = qBasis^-1 * qShoulderLocal * qBasis
	// (One can do qBasis conjugate if qBasis is unit)
	versor qBasisInv;
	glm_quat_inv(qBasis, qBasisInv);

	versor tempA;
	glm_quat_mul(qShoulderLocal, qBasis, tempA);
	glm_quat_mul(qBasisInv, tempA, outBoneARot);

	// Similarly for bone B, but typically you also multiply by outBoneARot
	// because the elbow’s rotation is relative to the shoulder.
	versor tempB;
	glm_quat_mul(qElbowLocal, qBasis, tempB);
	glm_quat_mul(qBasisInv, tempB, tempB);
	// If you want the elbow rotation in *world* space, that’s the final.
	// But to get the local relative rotation of bone B, you often do:
	glm_quat_mul(tempB, outBoneARot, outBoneBRot);
	// outBoneBRot is the elbow’s rotation in world, relative to the root,
	// or you might do it a bit differently if your skeleton expects local angles.

	// That’s it!  outBoneARot and outBoneBRot are the final quaternions.
}

void retarget_ik(RetargetConfig config, Skeleton *srcSkeleton, HumanJointMap *srcJointMap, Skeleton *dstSkeleton, HumanJointMap *dstJointMap, Animation *anim) {
	int srcHandJointIndex = skeleton_index_of_human_joint(srcSkeleton, srcJointMap, HumanJoint_HandR);
	vec3 target = {0};
	glm_mat4_mulv3(srcSkeleton->joints[srcHandJointIndex].animatedWorldTransform, target, 1.0f, target);

	int handJointIndexR = skeleton_index_of_human_joint(dstSkeleton, dstJointMap, HumanJoint_HandR);
	int elbowJointIndexR = skeleton_index_of_human_joint(dstSkeleton, dstJointMap, HumanJoint_LowerArmR);
	int rootJointIndexR = skeleton_index_of_human_joint(dstSkeleton, dstJointMap, HumanJoint_UpperArmR);

	vec3 hand = {0};
	glm_mat4_mulv3(dstSkeleton->joints[handJointIndexR].animatedWorldTransform, hand, 1.0f, hand);

	vec3 elbow = {0};
	glm_mat4_mulv3(dstSkeleton->joints[elbowJointIndexR].animatedWorldTransform, elbow, 1.0f, elbow);

	vec3 root = {0};
	glm_mat4_mulv3(dstSkeleton->joints[rootJointIndexR].animatedWorldTransform, root, 1.0f, root);

    // these output rotations are relative to the joint forward axis (fwd=joint->child)
	versor outRootRot;
	versor outMidRot;
	two_bone_ik(root, elbow, hand, target, (vec3){-1.0f, 1.0f, 1.0f}, outRootRot, outMidRot);

    // take the delta rotations and transform them from world space to parent space
    // then apply them to the joints

	// todo these are world space rotations. calculate the delta and apply them, then update the bind transforms. done!

    /* begin draw hands */
    vec3 drawOffset = {1.0f, 0.0f, 0.0f};

    vec3 drawHand;
    glm_vec3_copy(hand, drawHand);
    glm_vec3_add(drawHand, drawOffset, drawHand);

    draw_sphere(drawHand, 0.025f, 4, 4, (vec3){1.0f, 1.0f, 0.0f});

    vec3 drawRoot;
    glm_vec3_copy(root, drawRoot);
    glm_vec3_add(drawRoot, drawOffset, drawRoot);

    draw_sphere(drawRoot, 0.025f, 4, 4, (vec3){1.0f, 1.0f, 0.0f});
    /* end draw hands */

//    versor rootBaseRot;
//    glm_mat4_quat(dstSkeleton->joints[rootJointIndexR].animatedWorldTransform, rootBaseRot);
//
//    versor rootBaseRotInv;
//    glm_quat_inv(rootBaseRot, rootBaseRotInv);
//
//    // 2) Delta = IKout * inverse(base)
//    versor delta;
//    glm_quat_mul(outRootRot, rootBaseRotInv, delta); // so delta = ikRotWorld * baseInv
//
//    // 3) Final = base * delta
//    versor final;
//    glm_quat_mul(rootBaseRot, delta, final);

    // ROOT TO MID
    vec3 dir1;
    glm_vec3_sub(elbow, root, dir1);
    glm_vec3_normalize(dir1);

    vec3 dir1len;
    glm_vec3_mul(dir1, (vec3){0.25f, 0.25f, 0.25f}, dir1len);

    glm_quat_rotatev(outRootRot, dir1len, dir1len);

    vec3 d1;
    glm_vec3_copy(root, d1);
    glm_vec3_add(d1, drawOffset, d1);

    draw_sphere(d1, 0.04f, 4, 4, (vec3){1.0f, 1.0f, 1.0f});

    vec3 d2;
    glm_vec3_copy(d1, d2);
    glm_vec3_add(d2, dir1len, d2);
    //glm_vec3_add(d2, (vec3){0.0f, 0.06f, 0.0f}, d2);

    draw_sphere(d2, 0.04f, 4, 4, (vec3){1.0f, 1.0f, 1.0f});

    draw_line(d1, d2, (vec3){1.0f, 1.0f, 1.0f});



    ///// ACTUALLY DO IT
    mat4 rootParentWorldToLocal;
    glm_mat4_inv(dstSkeleton->joints[dstSkeleton->joints[rootJointIndexR].parent_index].animatedWorldTransform, rootParentWorldToLocal);

    mat4 outRootRotMat4;
    glm_quat_mat4(outRootRot, outRootRotMat4);

    mat4 outRootRotInParentLocalSpace;
    glm_mat4_mulN((mat4 *[]){&rootParentWorldToLocal, &outRootRotMat4}, 2, outRootRotInParentLocalSpace);
}

float skeleton_compute_scale_from_source_to_target(Skeleton *src, Skeleton *dst) {
    return dst->center_of_mass[1] / src->center_of_mass[1];
}

void retarget(RetargetConfig config, Skeleton *srcSkeleton, HumanJointMap *srcJointMap, Skeleton *dstSkeleton, HumanJointMap *dstJointMap, Animation *src, Animation *dst, Arena *arena) {
	dst->channels = arena_alloc_zeroed(arena, sizeof(AnimationChannel) * src->num_channels);

    float retarget_scale = skeleton_compute_scale_from_source_to_target(srcSkeleton, dstSkeleton);
	for (int i = 0; i < src->num_channels; i++) {
		AnimationChannel *channel = &src->channels[i];

		HumanJoint humanJoint = HumanJoint_None;
		const char *humanJointName;
		for (int j = 0; j < srcJointMap->count; j++) {
			if (strcmp(channel->name, srcJointMap->mappings[j].sourceName) == 0) {
				humanJoint = srcJointMap->mappings[j].targetJoint;
				humanJointName = humanJointToString(humanJoint);
				break;
			}
		}

		if (humanJoint == HumanJoint_None) {
			continue;
		}

		// scale not supported
		if (channel->path_type == cgltf_animation_path_type_scale) {
			continue;
		}

		bool translationTransfer = false;
		if (config.allow_root_motion && humanJoint == HumanJoint_Root) {
			translationTransfer = true;
		}
		if (config.allow_pelvis_motion && humanJoint == HumanJoint_Pelvis) {
			translationTransfer = true;
		}
		if (channel->path_type == cgltf_animation_path_type_translation && !translationTransfer) {
			continue;
		}

		AnimationChannel *dstChannel = &dst->channels[dst->num_channels];
		dst->num_channels++;

		// find the target joint name from the human joint
		const char *dstJointName;
		for (int j = 0; j < dstJointMap->count; j++) {
			JointMapping mapping = dstJointMap->mappings[j];
			if (mapping.targetJoint == humanJoint) {
				dstJointName = mapping.sourceName;
				break;
			}
		}
		if (dstJointName == NULL) {
			fprintf(stderr, "Missing target joint mapping for joint %s\n", humanJointName);
			exit(1);
		}

		strncpy(dstChannel->name, dstJointName, 127);
		dstChannel->name[127] = '\0';
		dstChannel->frames = arena_alloc_zeroed(arena, sizeof(AnimationFrame) * channel->frame_count);
		dstChannel->frame_count = channel->frame_count;
		dstChannel->times = arena_alloc_zeroed(arena, sizeof(float) * channel->frame_count);
		memcpy(dstChannel->times, channel->times, sizeof(float) * channel->frame_count);
		dstChannel->path_type = channel->path_type;

		// find the source joint index
		int sourceJointIndex = -1;
		for (int j = 0; j < srcSkeleton->count; j++) {
			if (strcmp(channel->name, srcSkeleton->joints[j].name) == 0) {
				sourceJointIndex = j;
				break;
			}
		}
		if (sourceJointIndex == -1) {
			fprintf(stderr, "Can't find source joint index for joint %s\n", humanJointName);
			exit(1);
		}

		int targetJointIndex = -1;
		for (int j = 0; j < dstSkeleton->count; j++) {
			if (strcmp(dstJointName, dstSkeleton->joints[j].name) == 0) {
				targetJointIndex = j;
				break;
			}
		}
		if (targetJointIndex == -1) {
			fprintf(stderr, "Can't find target joint index for joint %s\n", humanJointName);
			exit(1);
		}

		mat4 parentLocalToWorld;
		glm_mat4_identity(parentLocalToWorld);
		mat4 targetParentWorldToLocal;
		glm_mat4_identity(targetParentWorldToLocal);
		if (humanJoint != HumanJoint_Root) {
			glm_mat4_copy(srcSkeleton->joints[srcSkeleton->joints[sourceJointIndex].parent_index].bindPose, parentLocalToWorld);
			glm_mat4_copy(dstSkeleton->joints[dstSkeleton->joints[targetJointIndex].parent_index].inverseBindPose, targetParentWorldToLocal);
		}
		mat4 sourceWorldToLocal;
		glm_mat4_copy(srcSkeleton->joints[sourceJointIndex].inverseBindPose, sourceWorldToLocal);
		mat4 targetLocalToWorld;
		glm_mat4_copy(dstSkeleton->joints[targetJointIndex].bindPose, targetLocalToWorld);
		for (int j = 0; j < channel->frame_count; j++) {
			AnimationFrame *frame = &channel->frames[j];
			AnimationFrame *dstFrame = &dstChannel->frames[j];

			// todo remove: this makes no sense, we only support root/pelvis translation and those don't need 'retargeting' anyway, just scaling or other techniques
//			if (channel->path_type == cgltf_animation_path_type_translation) {
//				// convert the local space animated transform to world space
//				mat4 localTranslationTransform;
//				glm_translate_make(localTranslationTransform, frame->translation);
//
//				mat4 worldTranslationTransform;
//				glm_mat4_mul(parentLocalToWorld, localTranslationTransform, worldTranslationTransform);
//
//				mat4 delta;
//				glm_mat4_mul(worldTranslationTransform, sourceWorldToLocal, delta);
//
//				// target bone -> to world space -> apply delta -> back to local space (right to left)
//				mat4 retargeted;
//				glm_mat4_mulN((mat4 *[]){&targetParentWorldToLocal, &delta, &targetLocalToWorld}, 3, retargeted);
//
//				vec3 T = {0};
//				glm_mat4_mulv3(retargeted, T, 1.0f, T);
//
//				if (retarget_config.pelvis_motion_space == PelvisMotionSpace_Source) {
//					// do nothing extra
//				} else if (retarget_config.pelvis_motion_space == PelvisMotionSpace_CenterOfMass) {
//					vec3 source_new_center_of_mass = {0};
//					glm_vec3_add(frame->translation, srcSkeleton->pelvis_to_center_of_mass_offset, source_new_center_of_mass);
//
//					vec3 target_new_center_of_mass = {0};
//					glm_vec3_add(T, dstSkeleton->pelvis_to_center_of_mass_offset, target_new_center_of_mass);
//
//					// some skeletons have larger or smaller distances from pelvis to center of mass.
//					// for example: skeleton A pelvis is much lower than CoM, and skeleton B pelvis is much closer to CoM.
//					// if skeleton A sits on the floor and the pelvis Y is now zero, retargeting this Y translation to skeleton B means skeleton B will be sitting under the floor.
//					// to correct for this, we scale the Y translation up to be larger to accomodate for skeleton B's higher pelvis position.
//
//					// to correct for this, we calculate the difference between the 2 distances to center of mass and add it
//					vec3 src_distance_to_com_minus_dst_distance_to_com = {0};
//					glm_vec3_sub(srcSkeleton->pelvis_to_center_of_mass_offset, dstSkeleton->pelvis_to_center_of_mass_offset, src_distance_to_com_minus_dst_distance_to_com);
//					printf("it is %f\n", src_distance_to_com_minus_dst_distance_to_com[1]);
//
//					T[1] += src_distance_to_com_minus_dst_distance_to_com[1] / retarget_scale;
//
//					vec3 scale_pelvis_to_center_of_mass = {0};
//					glm_vec3_div(srcSkeleton->pelvis_to_center_of_mass_offset, dstSkeleton->pelvis_to_center_of_mass_offset, scale_pelvis_to_center_of_mass);
//				}
//
//                // scale translation to target skeleton size
//                if (retarget_config.scale_pelvis_motion_to_target == 1) {
//                    //glm_vec3_mul(T, (vec3){retarget_scale, retarget_scale, retarget_scale}, T);
//                }
//				printf("T is %f orig %f\n", T[1], frame->translation[1]);
//
//				glm_vec3_copy(T, dstFrame->translation);
//			}

			if (channel->path_type == cgltf_animation_path_type_translation) {
				vec3 T = { frame->translation[0], frame->translation[1], frame->translation[2]};

				if (retarget_config.pelvis_motion_space == PelvisMotionSpace_Source) {
					// do nothing extra
				} else if (retarget_config.pelvis_motion_space == PelvisMotionSpace_CenterOfMass) {
					vec3 source_new_center_of_mass = {0};
					glm_vec3_add(frame->translation, srcSkeleton->pelvis_to_center_of_mass_offset, source_new_center_of_mass);

					vec3 target_new_center_of_mass = {0};
					glm_vec3_add(T, dstSkeleton->pelvis_to_center_of_mass_offset, target_new_center_of_mass);

					// some skeletons have larger or smaller distances from pelvis to center of mass.
					// for example: skeleton A pelvis is much lower than CoM, and skeleton B pelvis is much closer to CoM.
					// if skeleton A sits on the floor and the pelvis Y is now zero, retargeting this Y translation to skeleton B means skeleton B will be sitting under the floor.
					// to correct for this, we scale the Y translation up to be larger to accomodate for skeleton B's higher pelvis position.

					// to correct for this, we calculate the difference between the 2 distances to center of mass and add it
					vec3 src_distance_to_com_minus_dst_distance_to_com = {0};
					glm_vec3_sub(srcSkeleton->pelvis_to_center_of_mass_offset, dstSkeleton->pelvis_to_center_of_mass_offset, src_distance_to_com_minus_dst_distance_to_com);
					printf("it is %f\n", src_distance_to_com_minus_dst_distance_to_com[1]);

					T[1] += src_distance_to_com_minus_dst_distance_to_com[1] / retarget_scale;

					vec3 scale_pelvis_to_center_of_mass = {0};
					glm_vec3_div(srcSkeleton->pelvis_to_center_of_mass_offset, dstSkeleton->pelvis_to_center_of_mass_offset, scale_pelvis_to_center_of_mass);
				}

				// scale translation to target skeleton size
				if (retarget_config.scale_pelvis_motion_to_target == 1) {
					glm_vec3_mul(T, (vec3){retarget_scale, retarget_scale, retarget_scale}, T);
				}

				glm_vec3_copy(T, dstFrame->translation);
			}

			if (channel->path_type == cgltf_animation_path_type_rotation) {
				// convert the local space transform to world space
				mat4 localRotationTransform;
				glm_quat_mat4(frame->rotation, localRotationTransform);

				mat4 worldRotationTransform;
				glm_mat4_mul(parentLocalToWorld, localRotationTransform, worldRotationTransform);

				mat4 delta;
				glm_mat4_mul(worldRotationTransform, sourceWorldToLocal, delta);

				mat4 retargeted;
				glm_mat4_mulN((mat4 *[]){&targetParentWorldToLocal, &delta, &targetLocalToWorld}, 3, retargeted);

				versor r;
				glm_mat4_quat(retargeted, r);
				glm_quat_copy(r, dstFrame->rotation);
			}
		}
	}
}

void draw_ik_targets(Skeleton *skeleton, HumanJointMap *jointMap, vec3 offset) {
	int handJointIndexR = skeleton_index_of_human_joint(skeleton, jointMap, HumanJoint_HandR);
	int handJointIndexL = skeleton_index_of_human_joint(skeleton, jointMap, HumanJoint_HandL);

	vec3 outR = {0};
	glm_mat4_mulv3(skeleton->joints[handJointIndexR].animatedWorldTransform, outR, 1.0f, outR);
	glm_vec3_add(outR, offset, outR);

	vec3 outL = {0};
	glm_mat4_mulv3(skeleton->joints[handJointIndexL].animatedWorldTransform, outL, 1.0f, outL);
	glm_vec3_add(outL, offset, outL);

	draw_sphere(outR, 0.025f, 4, 4, (vec3){1.0f, 0.0f, 0.0f});
	draw_sphere(outL, 0.025f, 4, 4, (vec3){1.0f, 0.0f, 0.0f});
}

void draw_player(int frame_index) {
	int width;
	int height;
	glfwGetWindowSize(window, &width, &height);

	int panelWidth = 225;
	int panelHeight = 32;

	if (nk_begin(nk_ctx, "Player",
				 nk_rect((width - panelWidth) / 2, height - panelHeight - 4, panelWidth, panelHeight),
				 NK_WINDOW_BORDER | NK_WINDOW_NO_SCROLLBAR)) {
		nk_layout_row_begin(nk_ctx, NK_STATIC, 22, 2);

//		nk_layout_row_push(nk_ctx, 160);
//		if (nk_group_begin(nk_ctx, "FrameGroup", NK_WINDOW_NO_SCROLLBAR)) {
//			nk_layout_row_begin(nk_ctx, NK_STATIC, 16, 3);
//			{
//				nk_layout_row_push(nk_ctx, 72);
//				char frame_str[128];
//				snprintf(frame_str, sizeof(frame_str), "Frame: %d", frame_index);
//				frame_str[127] = '\0';
//				nk_label(nk_ctx, frame_str, NK_TEXT_ALIGN_LEFT);
//				nk_layout_row_push(nk_ctx, 24);
//				nk_button_label(nk_ctx, "< ");
//				nk_layout_row_push(nk_ctx, 24);
//				nk_button_label(nk_ctx, "> ");
//			}
//			nk_layout_row_end(nk_ctx);
//		}
//		nk_group_end(nk_ctx);

		nk_layout_row_push(nk_ctx, 120);
		nk_property_float(nk_ctx, "Speed", 0.05f, &global.playback_rate, 2.0f, 0.25f, 0.05f);

		nk_layout_row_push(nk_ctx, 72);
		if (nk_button_label(nk_ctx, global.animation_playing ? "Pause" : "Play")) {
			global.animation_playing = !global.animation_playing;
		}

		nk_layout_row_end(nk_ctx);
	}
	nk_end(nk_ctx);
}

/* unused */
void draw_timeline() {
	int width;
	int height;
	glfwGetWindowSize(window, &width, &height);

	int panelWidth = 600;
	int panelHeight = 32;

	//struct nk_rect space;
	//enum nk_widget_layout_states state;
	//state = nk_widget(&space, ctx);
	//if (!state) return;

	int TOTAL_FRAMES = 120;
	float frame_spacing = 12;//space.w / (float)(TOTAL_FRAMES - 1);
	int totalWidth = TOTAL_FRAMES * frame_spacing;

	// no padding
	struct nk_style_window original_style = nk_ctx->style.window;
	nk_ctx->style.window.padding = nk_vec2(0, 0);
	nk_ctx->style.window.group_padding = nk_vec2(0, 0);

	if (nk_begin(nk_ctx, "Timeline",
				 nk_rect((width - panelWidth) / 2, height - panelHeight - 4, panelWidth, panelHeight),
				 NK_WINDOW_BORDER | NK_WINDOW_NO_SCROLLBAR)) {
		nk_layout_row_static(nk_ctx, panelHeight, panelWidth, 1);

		if (nk_group_begin(nk_ctx, "TimelineGroup", 0)) {
			//nk_layout_row_dynamic(nk_ctx, panelHeight, 1);
			struct nk_rect row_bounds = nk_widget_bounds(nk_ctx);
			nk_layout_row_static(nk_ctx, row_bounds.h, totalWidth, 1);

			//nk_label(nk_ctx, "This is a bottom-centered panel.", NK_TEXT_CENTERED);
			//nk_layout_row_dynamic(nk_ctx, 30, 1);
			//nk_label(nk_ctx, "This is a bottom-centered panel.", NK_TEXT_CENTERED);

			struct nk_command_buffer *canvas;
			//struct nk_input *input = &ctx->input;
			canvas = nk_window_get_canvas(nk_ctx);

			struct nk_rect space;
			nk_widget(&space, nk_ctx);

			int TOTAL_FRAMES = 120;
			// Calculate spacing between frames
			float frame_spacing = 12;//space.w / (float)(TOTAL_FRAMES - 1);

			// Draw frame indicators
			for (int i = 0; i < TOTAL_FRAMES; ++i) {
				float x = space.x + i * frame_spacing;
				float radius = 6.0f;
				float y = space.y + ((space.h - radius) / 2);

				nk_fill_circle(canvas, nk_rect(x, y, radius, radius), nk_rgb(255, 255, 0));
			}
		}
		nk_group_end(nk_ctx);
	}
	nk_end(nk_ctx);
	nk_ctx->style.window = original_style;
}

void draw_fps() {
	int width;
	int height;
	glfwGetWindowSize(window, &width, &height);

	if (nk_begin(nk_ctx, "FPS", nk_rect(width - 128, 12, 96, 22),NK_WINDOW_NO_SCROLLBAR)) {
		nk_layout_row_dynamic(nk_ctx, 16, 1);

		char label1[128];
		snprintf(label1, sizeof(label1), "fps %d", global.fps);
		nk_label(nk_ctx, label1, NK_TEXT_ALIGN_LEFT);
	}
	nk_end(nk_ctx);
}

void draw_pose_legend_ui() {
	if (nk_begin(nk_ctx, "Legend", nk_rect(356, 64, 156, 96),NK_WINDOW_TITLE | NK_WINDOW_NO_SCROLLBAR)) {
		float row1_widths[] = {42, 128};
		nk_layout_row(nk_ctx, NK_STATIC, 16, 2, row1_widths);

		nk_label_colored(nk_ctx, "-----", NK_TEXT_LEFT, nk_rgb(0,255,0));
		nk_label(nk_ctx, "Adjusted pose", NK_TEXT_ALIGN_LEFT);

		nk_label_colored(nk_ctx, "-----", NK_TEXT_LEFT, nk_rgb(255,255,0));
		nk_label(nk_ctx, "Original pose", NK_TEXT_ALIGN_LEFT);

        nk_label_colored(nk_ctx, "    0", NK_TEXT_LEFT, nk_rgb(60,60,255));
        nk_label(nk_ctx, "Center of Mass", NK_TEXT_ALIGN_LEFT);
	}
	nk_end(nk_ctx);
}

void draw_ik_legend_ui() {
	if (nk_begin(nk_ctx, "Legend", nk_rect(356, 64, 156, 52),NK_WINDOW_TITLE | NK_WINDOW_NO_SCROLLBAR)) {
		float row1_widths[] = {12, 128};
		nk_layout_row(nk_ctx, NK_STATIC, 16, 2, row1_widths);

		nk_label_colored(nk_ctx, "0", NK_TEXT_LEFT, nk_rgb(255,0,0));
		nk_label(nk_ctx, "IK Target", NK_TEXT_ALIGN_LEFT);
	}
	nk_end(nk_ctx);
}

void draw_ui() {
	enum {EASY, HARD};
	static int op = EASY;
	static float value = 0.6f;
	static int i = 20;

	if (nk_begin(nk_ctx, "Retargeter", nk_rect(12, 12, 320, 480),
				 NK_WINDOW_BORDER|NK_WINDOW_MOVABLE)) {
		nk_layout_row_static(nk_ctx, 16, 180, 1);

		char label1[128];
		snprintf(label1, sizeof(label1), "Animation: %s", "run_anim");
		nk_label(nk_ctx, label1, NK_TEXT_ALIGN_LEFT);

		// spacer
		nk_layout_row_static(nk_ctx, 16, 180, 1);

		nk_layout_row_static(nk_ctx, 16, 180, 1);
		nk_label(nk_ctx, "Stage 1: Bind pose", NK_TEXT_ALIGN_LEFT);

		{
			struct nk_rect bounds = nk_widget_bounds(nk_ctx);
			retarget_config.auto_tpose = nk_check_label(nk_ctx, "Enable pose matching", retarget_config.auto_tpose == 1);
			if (nk_input_is_mouse_hovering_rect(&nk_ctx->input, bounds))
				nk_tooltip(nk_ctx, "Match the target skeleton to the source skeleton pose.");
		}

		nk_layout_row_static(nk_ctx, 16, 180, 1);
		if (nk_button_label(nk_ctx, "View bind pose")) {
			mode = Mode_Pose;
		}

		// spacer
		nk_layout_row_static(nk_ctx, 32, 180, 1);

		nk_layout_row_static(nk_ctx, 16, 190, 1);
		nk_label(nk_ctx, "Stage 2: Transfer animation", NK_TEXT_ALIGN_LEFT);

		nk_layout_row_dynamic(nk_ctx, 16, 1);
		retarget_config.allow_root_motion = nk_check_label(nk_ctx, "Allow root motion", retarget_config.allow_root_motion == 1);
		nk_layout_row_dynamic(nk_ctx, 16, 1);
		retarget_config.allow_pelvis_motion = nk_check_label(nk_ctx, "Allow pelvis motion", retarget_config.allow_pelvis_motion == 1);
		nk_layout_row_dynamic(nk_ctx, 16, 1);
		retarget_config.scale_pelvis_motion_to_target = nk_check_label(nk_ctx, "Scale pelvis motion to target", retarget_config.scale_pelvis_motion_to_target == 1);

		nk_layout_row_dynamic(nk_ctx, 16, 2);
		nk_label(nk_ctx, "Pelvis motion space", NK_TEXT_LEFT);
		if (nk_combo_begin_label(nk_ctx, PelvisMotionSpaceName[retarget_config.pelvis_motion_space], nk_vec2(nk_widget_width(nk_ctx), 200))) {
			int i;
			nk_layout_row_dynamic(nk_ctx, 25, 1);
			for (i = 0; i < PelvisMotionSpace_End; i++) {
				if (nk_combo_item_label(nk_ctx, PelvisMotionSpaceName[i], NK_TEXT_LEFT)){
					retarget_config.pelvis_motion_space = i;
				}
			}
			nk_combo_end(nk_ctx);
		}

		nk_layout_row_static(nk_ctx, 16, 180, 1);
		if (nk_button_label(nk_ctx, "View animation")) {
			mode = Mode_Default;
		}

		// spacer
		nk_layout_row_static(nk_ctx, 32, 180, 1);

		nk_layout_row_static(nk_ctx, 16, 180, 1);
		nk_label(nk_ctx, "Stage 3: IK", NK_TEXT_ALIGN_LEFT);
		retarget_config.enable_hand_ik = nk_check_label(nk_ctx, "Enable Hand IK", retarget_config.enable_hand_ik == 1);
		retarget_config.enable_foot_ik = nk_check_label(nk_ctx, "Enable Foot IK", retarget_config.enable_foot_ik == 1);
		nk_layout_row_static(nk_ctx, 16, 180, 1);
		if (nk_button_label(nk_ctx, "View IK")) {
			mode = Mode_IK;
		}

		// spacer
		nk_layout_row_static(nk_ctx, 32, 180, 1);

		nk_layout_row_static(nk_ctx, 16, 180, 1);
		nk_label(nk_ctx, "Stage 3: Final", NK_TEXT_ALIGN_LEFT);
		nk_layout_row_static(nk_ctx, 16, 180, 1);
		if (nk_button_label(nk_ctx, "View final animation")) {
			mode = Mode_Default;
		}

		/* fixed widget window ratio width */
//		nk_layout_row_dynamic(nk_ctx, 30, 2);
//		if (nk_option_label(nk_ctx, "easy", op == EASY)) op = EASY;
//		if (nk_option_label(nk_ctx, "hard", op == HARD)) op = HARD;
//
//		/* custom widget pixel width */
//		nk_layout_row_begin(nk_ctx, NK_STATIC, 30, 2);
//		{
//			nk_layout_row_push(nk_ctx, 50);
//			nk_label(nk_ctx, "Volume:", NK_TEXT_LEFT);
//			nk_layout_row_push(nk_ctx, 110);
//			nk_slider_float(nk_ctx, 0, &value, 1.0f, 0.1f);
//		}
//		nk_layout_row_end(nk_ctx);
	}
	nk_end(nk_ctx);
}

void pose_match(Skeleton *sourceSkeleton, HumanJointMap *sourceJointMap, Skeleton *targetSkeleton, HumanJointMap *targetJointMap) {
	/* RIGHT ARMS */
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_ClavicleR, HumanJoint_UpperArmR);
	skeleton_compute_bind_pose(targetSkeleton);

	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_UpperArmR, HumanJoint_LowerArmR);
	skeleton_compute_bind_pose(targetSkeleton);

	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_LowerArmR, HumanJoint_HandR);
	skeleton_compute_bind_pose(targetSkeleton);

	/* LEFT ARMS */
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_ClavicleL, HumanJoint_UpperArmL);
	skeleton_compute_bind_pose(targetSkeleton);

	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_UpperArmL, HumanJoint_LowerArmL);
	skeleton_compute_bind_pose(targetSkeleton);

	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_LowerArmL, HumanJoint_HandL);
	skeleton_compute_bind_pose(targetSkeleton);


	/* RIGHT FINGERS */
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_HandR, HumanJoint_FingerIndexR1);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerIndexR1, HumanJoint_FingerIndexR2);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerIndexR2, HumanJoint_FingerIndexR3);
	skeleton_compute_bind_pose(targetSkeleton);

	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_HandR, HumanJoint_FingerMiddleR1);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerMiddleR1, HumanJoint_FingerMiddleR2);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerMiddleR2, HumanJoint_FingerMiddleR3);
	skeleton_compute_bind_pose(targetSkeleton);

	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_HandR, HumanJoint_FingerRingR1);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerRingR1, HumanJoint_FingerRingR2);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerRingR2, HumanJoint_FingerRingR3);
	skeleton_compute_bind_pose(targetSkeleton);

	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_HandR, HumanJoint_FingerPinkyR1);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerPinkyR1, HumanJoint_FingerPinkyR2);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerPinkyR2, HumanJoint_FingerPinkyR3);
	skeleton_compute_bind_pose(targetSkeleton);

	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_HandR, HumanJoint_FingerThumbR1);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerThumbR1, HumanJoint_FingerThumbR2);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerThumbR2, HumanJoint_FingerThumbR3);
	skeleton_compute_bind_pose(targetSkeleton);

	/* LEFT FINGERS */
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_HandL, HumanJoint_FingerIndexL1);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerIndexL1, HumanJoint_FingerIndexL2);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerIndexL2, HumanJoint_FingerIndexL3);
	skeleton_compute_bind_pose(targetSkeleton);

	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_HandL, HumanJoint_FingerMiddleL1);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerMiddleL1, HumanJoint_FingerMiddleL2);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerMiddleL2, HumanJoint_FingerMiddleL3);
	skeleton_compute_bind_pose(targetSkeleton);

	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_HandL, HumanJoint_FingerRingL1);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerRingL1, HumanJoint_FingerRingL2);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerRingL2, HumanJoint_FingerRingL3);
	skeleton_compute_bind_pose(targetSkeleton);

	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_HandL, HumanJoint_FingerPinkyL1);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerPinkyL1, HumanJoint_FingerPinkyL2);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerPinkyL2, HumanJoint_FingerPinkyL3);
	skeleton_compute_bind_pose(targetSkeleton);

	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_HandL, HumanJoint_FingerThumbL1);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerThumbL1, HumanJoint_FingerThumbL2);
	skeleton_compute_bind_pose(targetSkeleton);
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_FingerThumbL2, HumanJoint_FingerThumbL3);
	skeleton_compute_bind_pose(targetSkeleton);


	/* RIGHT LEGS */
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_ThighR, HumanJoint_CalfR);
	skeleton_compute_bind_pose(targetSkeleton);

	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_CalfR, HumanJoint_FootR);
	skeleton_compute_bind_pose(targetSkeleton);

	/* LEFT LEGS */
	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_ThighL, HumanJoint_CalfL);
	skeleton_compute_bind_pose(targetSkeleton);

	pose_match_joint(sourceSkeleton, sourceJointMap, targetSkeleton, targetJointMap, HumanJoint_CalfL, HumanJoint_FootL);
	skeleton_compute_bind_pose(targetSkeleton);
}

void auto_tpose(Skeleton *skeleton, HumanJointMap *jointMap) {
    tpose_bone_align(skeleton, jointMap, HumanJoint_ClavicleR, HumanJoint_UpperArmR, (vec3){-1.0f, 0.0f, 0.0f});
	skeleton_compute_bind_pose(skeleton);

    tpose_bone_align(skeleton, jointMap, HumanJoint_UpperArmR, HumanJoint_LowerArmR, (vec3){-1.0f, 0.0f, 0.0f});
	skeleton_compute_bind_pose(skeleton);

    tpose_bone_align(skeleton, jointMap, HumanJoint_LowerArmR, HumanJoint_HandR, (vec3){-1.0f, 0.0f, 0.0f});
	skeleton_compute_bind_pose(skeleton);

    tpose_bone_align(skeleton, jointMap, HumanJoint_ClavicleL, HumanJoint_UpperArmL, (vec3){1.0f, 0.0f, 0.0f});
	skeleton_compute_bind_pose(skeleton);

    tpose_bone_align(skeleton, jointMap, HumanJoint_UpperArmL, HumanJoint_LowerArmL, (vec3){1.0f, 0.0f, 0.0f});
	skeleton_compute_bind_pose(skeleton);

    tpose_bone_align(skeleton, jointMap, HumanJoint_LowerArmL, HumanJoint_HandL, (vec3){1.0f, 0.0f, 0.0f});
	skeleton_compute_bind_pose(skeleton);

    tpose_bone_align(skeleton, jointMap, HumanJoint_ThighL, HumanJoint_CalfL, (vec3){0.0f, -1.0f, 0.0f});
	skeleton_compute_bind_pose(skeleton);

    tpose_bone_align(skeleton, jointMap, HumanJoint_CalfL, HumanJoint_FootL, (vec3){0.0f, -1.0f, 0.0f});
	skeleton_compute_bind_pose(skeleton);

    tpose_bone_align(skeleton, jointMap, HumanJoint_ThighR, HumanJoint_CalfR, (vec3){0.0f, -1.0f, 0.0f});
	skeleton_compute_bind_pose(skeleton);

    tpose_bone_align(skeleton, jointMap, HumanJoint_CalfR, HumanJoint_FootR, (vec3){0.0f, -1.0f, 0.0f});
	skeleton_compute_bind_pose(skeleton);
}

int main() {
	load_and_dump();

	global_arena = arena_create(4 * 1000 * 1000);//4mb

	vec3 c = { 1, 2, 3 };
	glm_vec3_rotate(c, glm_rad(45), (vec3){1.0f, 0.0f, 0.0f});

//    char *sourceAnimationFile = "tests/anim.gltf";
//    char *sourceJointMapFile = "jointmap_mixamo.txt";

//    char *sourceAnimationFile = "tests/bowshot2.gltf";
//    char *sourceJointMapFile = "tests/jointmap_blink.txt";

//char *targetSkeletonFile = "target.gltf";
    //char *targetJointMapFile = "jointmap_unreal.txt";

    char *sourceAnimationFile = "tests/sitting_eat.gltf";
	//char *sourceAnimationFile = "tests/sitting_enter.gltf";
    char *sourceJointMapFile = "jointmap_unreal.txt";
    char *targetSkeletonFile = "tests/player.gltf";
    char *targetJointMapFile = "tests/jointmap_player.txt";

	HumanJointMap sourceJointMap = {0};
	if (loadJointMappingFile(sourceJointMapFile, &sourceJointMap) != 0) {
		fprintf(stderr, "Failed to load source joint map file ('%s')", sourceJointMapFile);
		return -1;
	}

	HumanJointMap targetJointMap = {0};
	if (loadJointMappingFile(targetJointMapFile, &targetJointMap) != 0) {
		fprintf(stderr, "Failed to load target joint map file ('%s')", targetJointMapFile);
		return -1;
	}

	Skeleton sourceSkeleton = {0};
	load_skeleton(&sourceSkeleton, sourceAnimationFile);
    skeleton_estimate_center_of_mass(&sourceSkeleton, &sourceJointMap, sourceSkeleton.center_of_mass, sourceSkeleton.pelvis_to_center_of_mass_offset);
	Skeleton targetSkeleton = {0};
	load_skeleton(&targetSkeleton, targetSkeletonFile);
    skeleton_estimate_center_of_mass(&targetSkeleton, &targetJointMap, targetSkeleton.center_of_mass, targetSkeleton.pelvis_to_center_of_mass_offset);

	Animation anim = {0};
	load_anim(&sourceSkeleton, &anim, global_arena);

	if (sourceSkeleton.cgltf_data->animations_count == 0) {
		fprintf(stderr, "Source file has no animations");
		return -1;
	}

	if (sourceSkeleton.cgltf_data->animations_count > 1) {
		fprintf(stderr, "Source file has more than 1 animation (%d)", sourceSkeleton.cgltf_data->animations_count);
		return -1;
	}

//	cgltf_options gltf_write_options = {0};
//	//cgltf_data* data = /* TODO must be valid data */;
//	cgltf_result gltf_write_result = cgltf_write_file(&gltf_write_options, "out.gltf", targetSkeleton.cgltf_data);
//	if (gltf_write_result != cgltf_result_success) {
//		fprintf(stderr, "Failed to write output file\n");
//		return -1;
//	}

//	cgltf_free(sourceSkeleton.cgltf_data);
//	cgltf_free(targetSkeleton.cgltf_data);

	if (!glfwInit()) {
        printf("glfw init failed");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#if __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    glfwSetErrorCallback(glfw_error_callback);

    window = glfwCreateWindow(1280, 720, "Anim Transfer", NULL, NULL);
    if (!window) {
        printf("glfw make window failed\n");
        return -1;
    }
    glfwMakeContextCurrent(window);

    int version = gladLoadGL(glfwGetProcAddress);
    if (version == 0) {
        printf("glad fail\n");
        return -1;
    }
    printf("opengl %d.%d\n", GLAD_VERSION_MAJOR(version), GLAD_VERSION_MINOR(version));

	create_line_buffer(&line_buffer);

	GLuint prog = create_program(vertex_shader_src, fragment_shader_src);

	GLuint vao, vbo;
	glGenVertexArrays(1,&vao);
	glGenBuffers(1,&vbo);
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER,vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(cube_vertices), cube_vertices, GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3*sizeof(float),(void*)0);
	glBindVertexArray(0);

	glEnable(GL_DEPTH_TEST);

	/* Nk */
    struct nk_glfw glfw = {0};
    struct nk_image img;
	nk_ctx = nk_glfw3_init(&glfw, window, NK_GLFW3_INSTALL_CALLBACKS);

    struct nk_font_atlas *atlas;
	nk_glfw3_font_stash_begin(&glfw, &atlas);
	nk_glfw3_font_stash_end(&glfw);

	double elapsed_time = 0.0f;
	double animation_time = 0.0f;
	double last_time = glfwGetTime();
	double last_frame_time = glfwGetTime();
	int max_fps = 60;
	int frame_count = 0;
    while (!glfwWindowShouldClose(window)) {
		double now_time = glfwGetTime();
		elapsed_time += now_time - last_time;
		last_time = now_time;

		if (now_time - last_frame_time < (1.0 / max_fps)) {
			continue;
		}
		double delta_time = now_time - last_frame_time;
		last_frame_time = now_time;

		global.fps = (int) ceil((double) frame_count / elapsed_time);
		frame_count++;

        glfwPollEvents();

		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
			if (!global.leftMouseDown) {
				global.leftMouseDown = true;
				glfwGetCursorPos(window, &global.lastMouseX, &global.lastMouseY);
			} else {
				double mouseX, mouseY;
				glfwGetCursorPos(window, &mouseX, &mouseY);

				float deltaX = (float)(mouseX - global.lastMouseX);
				float deltaY = (float)(mouseY - global.lastMouseY);

				float sensitivity = 0.4f;
				global.cameraYaw   += -deltaX * sensitivity;
				global.cameraPitch += deltaY * sensitivity;

				if (global.cameraPitch > 89.0f)  global.cameraPitch = 89.0f;
				if (global.cameraPitch < -89.0f) global.cameraPitch = -89.0f;

				global.lastMouseX = mouseX;
				global.lastMouseY = mouseY;
			}
		} else {
			global.leftMouseDown = false;
		}

        nk_glfw3_new_frame(&glfw);

		draw_fps();
        draw_ui();

		draw_grid();

		/////////////
		Arena *arena = arena_create(512 * 1000);//512kb
        skeleton_reset_to_bind_pose(&sourceSkeleton);

        // set joint transforms from cgltf nodes
        for (size_t j = 0; j < targetSkeleton.count; j++) {
            skeleton_set_local_transform_from_cgltf(&targetSkeleton, j, targetSkeleton.cgltf_data->skins[0].joints[j]);
        }
        skeleton_compute_bind_pose(&targetSkeleton);
        skeleton_reset_to_bind_pose(&targetSkeleton);

		if (global.animation_playing) {
			animation_time += delta_time * global.playback_rate;
		}

		if (mode == Mode_Default) {
			if (retarget_config.auto_tpose) {
				pose_match(&sourceSkeleton, &sourceJointMap, &targetSkeleton, &targetJointMap);
			}

			animation_sample(&sourceSkeleton, &anim, animation_time);
            animation_compute_final_transforms(&sourceSkeleton);
			draw_skeleton_animated(&sourceSkeleton, (vec3){-1.0, 0.0, 0.0});

			Animation *dstAnim = arena_alloc_zeroed(arena, sizeof(Animation));
			retarget(retarget_config, &sourceSkeleton, &sourceJointMap, &targetSkeleton, &targetJointMap, &anim, dstAnim, arena);

			skeleton_reset_to_bind_pose(&targetSkeleton);
			animation_sample(&targetSkeleton, dstAnim, animation_time);
            animation_compute_final_transforms(&targetSkeleton);
            draw_skeleton_animated(&targetSkeleton, (vec3){1.0, 0.0, 0.0});

			draw_player(420);
		} else if (mode == Mode_Pose) {
			draw_skeleton_bindpose_dashed(&targetSkeleton, (vec3){1.0, 0.0, 0.0});
			if (retarget_config.auto_tpose) {
				pose_match(&sourceSkeleton, &sourceJointMap, &targetSkeleton, &targetJointMap);
			}
			draw_skeleton_bindpose(&sourceSkeleton, (vec3){-1.0, 0.0, 0.0});
			draw_skeleton_bindpose(&targetSkeleton, (vec3){1.0, 0.0, 0.0});
            draw_skeleton_center_of_mass(&sourceSkeleton, &sourceJointMap, (vec3){-1.0, 0.0, 0.0});
            draw_skeleton_center_of_mass(&targetSkeleton, &targetJointMap, (vec3){1.0, 0.0, 0.0});

			draw_pose_legend_ui();
		} else if (mode == Mode_IK) {
			if (retarget_config.auto_tpose) {
				pose_match(&sourceSkeleton, &sourceJointMap, &targetSkeleton, &targetJointMap);
			}

			animation_sample(&sourceSkeleton, &anim, animation_time);
            animation_compute_final_transforms(&sourceSkeleton);
			draw_skeleton_animated(&sourceSkeleton, (vec3){-1.0, 0.0, 0.0});

			Animation *dstAnim = arena_alloc_zeroed(arena, sizeof(Animation));
			retarget(retarget_config, &sourceSkeleton, &sourceJointMap, &targetSkeleton, &targetJointMap, &anim, dstAnim, arena);

			skeleton_reset_to_bind_pose(&targetSkeleton);
			animation_sample(&targetSkeleton, dstAnim, animation_time);
            animation_compute_final_transforms(&targetSkeleton);
            retarget_ik(retarget_config, &sourceSkeleton, &sourceJointMap, &targetSkeleton, &targetJointMap, dstAnim);
			draw_skeleton_animated(&targetSkeleton, (vec3){1.0, 0.0, 0.0});

			draw_ik_targets(&sourceSkeleton, &sourceJointMap, (vec3){-1.0, 0.0, 0.0});
			draw_ik_targets(&sourceSkeleton, &sourceJointMap, (vec3){1.0, 0.0, 0.0});
			draw_player(420);

			draw_ik_legend_ui();
		}
		arena_free(arena);
		////////

        int width;
        int height;
        glfwGetWindowSize(window, &width, &height);

        int framebufferWidth;
        int framebufferHeight;
        glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);

		glViewport(0, 0, framebufferWidth, framebufferHeight);
		glClearColor(0.12f, 0.16f, 0.18f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		mat4 proj, view, model, mvp;
		float aspect = (float)width/(float)height;
		glm_perspective(glm_rad(45.0f), aspect, 0.1f, 100.0f, proj);

		float yawRad = glm_rad(global.cameraYaw);
		float pitchRad = glm_rad(global.cameraPitch);
		vec3 cameraPos = {
				global.cameraDistance * cosf(pitchRad) * sinf(yawRad),
				global.cameraDistance * sinf(pitchRad),
				global.cameraDistance * cosf(pitchRad) * cosf(yawRad)
		};
		glm_lookat(cameraPos, (vec3){0.0f,1.0f,0.0f}, (vec3){0.0f,1.0f,0.0f}, view);

		glm_mat4_identity(model);
		float time = (float)glfwGetTime();
		glm_rotate_y(model, time, model);

		glm_mat4_mulN((mat4 *[]){&proj, &view, &model}, 3, mvp);

		//// cube
//		glUseProgram(prog);
//		GLint mvpLoc = glGetUniformLocation(prog,"MVP");
//		glUniformMatrix4fv(mvpLoc,1,GL_FALSE,(float*)mvp);
//
//		glBindVertexArray(vao);
//		glDrawArrays(GL_TRIANGLES, 0, 36);
		/////

		// draw line buffer
		mat4 linesModel, linesMvp;
		glm_mat4_identity(linesModel);
		glm_mat4_mulN((mat4 *[]){&proj, &view, &linesModel}, 3, linesMvp);
		draw_line_buffer(&line_buffer, linesMvp);

        /* IMPORTANT: `nk_glfw_render` modifies some global OpenGL state
         * with blending, scissor, face culling, depth test and viewport and
         * defaults everything back into a default state.
         * Make sure to either a.) save and restore or b.) reset your own state after
         * rendering the UI. */
        nk_glfw3_render(&glfw, NK_ANTI_ALIASING_ON, MAX_VERTEX_BUFFER, MAX_ELEMENT_BUFFER);

        glfwSwapBuffers(window);
    }

	delete_line_buffer(&line_buffer);

    nk_glfw3_shutdown(&glfw);
    glfwDestroyWindow(window);
    glfwTerminate();
	arena_free(global_arena);
}