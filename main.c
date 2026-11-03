// Slendytubbies PS2 - unified single-file build (main.c)
// Merged from: steering.h, game.h, audio_voice.h, render.h, net.h,
// terrain_surface.h, transform.h, game.c, net.c, render.c, ps2_main.c
// Generated headers (../generated, ../tests) are NOT merged: they are produced
// by tools/export_*.py and must exist next to this file's parent directory.
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <kernel.h>
#include <sifrpc.h>
#include <loadfile.h>
#include <netman.h>
#include <ps2ip.h>
#include <gsKit.h>
#include <gsInline.h>
#include <dmaKit.h>
#include <iopcontrol.h>
#include <sbv_patches.h>
#include <libpad.h>
#include <audsrv.h>
#include <timer.h>

/* ===== steering.h ===== */
typedef struct { float position[3],rotation[4],speed; } SteeringBody;

static inline void steering_normalize(float q[4]) {
    float length=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    for(int i=0;i<4;i++)q[i]/=length;
}
static inline void steering_init(SteeringBody *body,const float position[3],const float rotation[4]) {
    memcpy(body->position,position,sizeof(body->position));
    memcpy(body->rotation,rotation,sizeof(body->rotation));
    steering_normalize(body->rotation);body->speed=0;
}
/* Unity LookRotation followed by shortest-path quaternion Slerp. Keeping the
 * local forward motion is essential: moving directly toward the goal removes
 * the original vacuum's circular path around its single waypoint. */
static inline void steering_step(SteeringBody *body,const float goal[3],float accel,float limit,float damping,float dt) {
    float x=goal[0]-body->position[0],y=goal[1]-body->position[1],z=goal[2]-body->position[2];
    if(x*x+y*y+z*z>1e-10f) {
        float yaw=atan2f(x,z)*.5f,pitch=atan2f(y,sqrtf(x*x+z*z))*.5f;
        float sy=sinf(yaw),cy=cosf(yaw),sp=sinf(pitch),cp=cosf(pitch);
        float target[4]={-sp*cy,cp*sy,sp*sy,cp*cy},dot=0;
        for(int i=0;i<4;i++)dot+=body->rotation[i]*target[i];
        if(dot<0){dot=-dot;for(int i=0;i<4;i++)target[i]=-target[i];}
        float t=fminf(1,dt*damping),a=1-t,b=t;
        if(dot<.9995f) {
            float angle=acosf(fminf(1,dot)),den=sinf(angle);
            a=sinf((1-t)*angle)/den;b=sinf(t*angle)/den;
        }
        for(int i=0;i<4;i++)body->rotation[i]=a*body->rotation[i]+b*target[i];
        steering_normalize(body->rotation);
    }
    body->speed=fminf(limit,body->speed+accel*accel*dt*60);
    const float *q=body->rotation;float step=body->speed*dt;
    body->position[0]+=2*(q[0]*q[2]+q[1]*q[3])*step;
    body->position[1]+=2*(q[1]*q[2]-q[0]*q[3])*step;
    body->position[2]+=(1-2*(q[0]*q[0]+q[1]*q[1]))*step;
}
/* Source vertices already contain their initial world transform. Apply only
 * the rotation delta about the original root, then the animated translation. */
static inline void steering_matrix(float m[16],const SteeringBody *body,const float origin[3],const float initial[4]) {
    const float *q=body->rotation;
    float x=-q[3]*initial[0]+q[0]*initial[3]-q[1]*initial[2]+q[2]*initial[1];
    float y=-q[3]*initial[1]+q[0]*initial[2]+q[1]*initial[3]-q[2]*initial[0];
    float z=-q[3]*initial[2]-q[0]*initial[1]+q[1]*initial[0]+q[2]*initial[3];
    float w=q[3]*initial[3]+q[0]*initial[0]+q[1]*initial[1]+q[2]*initial[2];
    const float rotation[16]={1-2*(y*y+z*z),2*(x*y+z*w),2*(x*z-y*w),0,
        2*(x*y-z*w),1-2*(x*x+z*z),2*(y*z+x*w),0,
        2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y),0,0,0,0,1};
    memcpy(m,rotation,sizeof(rotation));
    for(int row=0;row<3;row++)m[12+row]=body->position[row]-m[row]*origin[0]-m[4+row]*origin[1]-m[8+row]*origin[2];
}

/* ===== game.h ===== */
typedef struct { float x,y,z; } Vec3;
typedef struct { float x,y,z,u,v,nx,ny,nz; } Vertex;
typedef struct {
    uint32_t vertices,triangles,vertex_offset,index_offset,texture,flags,id;
    float min[3],max[3],color[3];
} Mesh;
typedef struct { uint32_t width,height,offset,bytes; } Texture;
typedef struct { float min[3],max[3]; } Box;
typedef struct { float x,y,z,width,height,type; } Tree;
typedef struct { uint32_t offset,samples; } Sound;
typedef struct { uint32_t vertices,triangles,vertex_offset,index_offset; float min[3],max[3]; } CollisionMesh;
typedef struct {
    uint32_t magic,version,meshes,textures,boxes,trees,width,height;
    uint32_t mesh_offset,texture_offset,box_offset,tree_offset,height_offset,splat_offset,config_offset;
    uint32_t sounds,sound_offset,collision_meshes,collision_offset,bytes;
} WorldHeader;
typedef struct {
    const unsigned char *blob; const WorldHeader *header;
    const Mesh *mesh; const Texture *texture; const Box *box; const Tree *tree;
    const uint16_t *height; const uint8_t *splat; const float *config; const Sound *sound;
    const CollisionMesh *collision;
} World;
enum { M_COLLIDE=1,M_ENEMY=2,M_DOOR=4,M_HOOVER=8,M_COLLECT=16,M_TREE=32,M_VIEW=64,M_CULL=256,M_DAY_TREE=512,M_CUTOUT=1024 };
enum { C_SPAWN=0,C_ENEMY=3,C_DOOR=6,C_HOOVER=9,C_WAY=12,C_CUSTARD=15,
    C_WALK=45,C_CROUCH=46,C_RUN=47,C_CHASE=48,C_FAST=49,C_SPRINT=50,
    C_FOLLOW=51,C_RANGE=52,C_DAMAGE=53,C_OFFSCREEN=54,C_VISIBLE=55,
    C_ENV=56,C_SCALE=77,C_GROUND=80,C_TREE=84,C_TITLE=85,C_MAP=86,
    C_SCARE=87,C_WON=88,C_DEAD=89,C_POPUP=90,C_FONT=94 };
enum { B_JUMP=1,B_LIGHT=2,B_CROUCH=4,B_RUN=8,B_MAP=16,B_PAUSE=32,B_CONFIRM=64,B_BACK=128,B_UP=256,B_DOWN=512,B_LEFT=1024,B_RIGHT=2048 };
enum { ST_MENU,ST_PLAY,ST_PAUSE,ST_DEAD,ST_WON,ST_HELP,ST_CREDITS,ST_NETWORK,ST_WINSCENE };
enum { MENU_MAIN,MENU_SETTINGS,MENU_SINGLE,MENU_MULTI,MENU_COOP,MENU_VERSUS };
enum { EV_PICKUP=1,EV_SWITCH=2,EV_SCARE=4,EV_DEATH=8,EV_ENEMY_SEEN=16,EV_ENEMY_SHIFT=32 };
typedef struct { float move_x,move_y,look_x,look_y; uint32_t held,pressed; int connected; } Input;
typedef struct { int active,flags; Vec3 feet; float yaw; } Peer;
typedef struct {
    Vec3 feet,enemy; SteeringBody hoover; float yaw,pitch,vy,eye,health,enemy_speed;
    float time,state_time,door_open,scare_time,next_scare;
    int state,menu,menu_page,quality,time_of_day,flashlight,map,crouching,grounded,moving;
    int collected,scare_image,events,enemy_visible,enemy_attacking,draw_grass;
    float pickup_time,next_enemy_shift;
    uint16_t custards;
    uint32_t random;
    int coop;      // 1 while playing a networked co-op match
    Peer peer[4];  // remote players, indexed by their server id
} Game;
int world_open(World *world,const void *blob,size_t length);
float world_height(const World *world,float x,float z);
Vec3 game_mesh_vertex(const World *world,const Game *game,const Mesh *mesh,const Vertex *vertex);
void game_enemy_matrix(const World *world,const Game *game,float matrix[16]);
void game_hoover_matrix(const World *world,const Game *game,float matrix[16]);
void game_init(Game *game);
void game_start(Game *game,const World *world);
void game_coop_merge(Game *game,const World *world,uint16_t mask);
void game_update(Game *game,const World *world,const Input *input,float dt);
void game_advance(Game *game,const World *world,const Input *input,float elapsed);
int world_occluded(const World *world,const Game *game,Vec3 from,Vec3 to);

/* ===== audio_voice.h ===== */
enum { AUDIO_ULAW=0x80000000u,AUDIO_COUNT=0x7fffffffu };
static inline int ulaw_decode(uint8_t byte) {
    unsigned code=(unsigned)(uint8_t)~byte;
    int value=(((code&15)<<3)+132)<<((code>>4)&7);
    return code&128?132-value:value-132;
}
static inline int voice_ulaw_sample(const uint8_t *data,unsigned samples,int *cursor,int loop) {
    int position=*cursor;
    if(position<0||!samples)return 0;
    if((unsigned)position>=samples){if(!loop){*cursor=-1;return 0;}position=0;}
    int sample=ulaw_decode(data[position++]);
    *cursor=(!loop&&(unsigned)position==samples)?-1:position;
    return sample;
}
/* One-shot cursor also represents whether the voice is still playing. */
static inline int voice_start_once(int *cursor,unsigned samples) {
    if(!samples||(*cursor>=0&&(unsigned)*cursor<samples))return 0;
    *cursor=0;return 1;
}
static inline int voice_start_ready(int *cursor,unsigned samples,uint64_t now,uint64_t audible_until) {
    return now>=audible_until&&voice_start_once(cursor,samples);
}
static inline int voice_sample(const int16_t *pcm,unsigned samples,int *cursor,int loop) {
    int position=*cursor;
    if(position<0||!samples)return 0;
    if((unsigned)position>=samples){if(!loop){*cursor=-1;return 0;}position=0;}
    int sample=pcm[position++];
    *cursor=(!loop&&(unsigned)position==samples)?-1:position;
    return sample;
}

/* ===== render.h ===== */
typedef struct { float x,y,z,u,v; uint8_t r,g,b,a; float q; unsigned cache_key; } DrawVertex;
void platform_clear(uint8_t r,uint8_t g,uint8_t b);
void platform_world(const World *world,int bank);
void platform_triangle(int texture,const DrawVertex *vertices);
void platform_sprite(int texture,float x,float y,float width,float height,float u,float v,float uw,float vh,uint32_t color);
void platform_finish(void);
void platform_profile(int phase);
void platform_blend(int mode);
void platform_cutout(int enabled);
void platform_fog(uint8_t r,uint8_t g,uint8_t b);
int platform_static_begin(int key);
void platform_static_end(void);
void platform_cache_reset(void);
void platform_work(int kind,int begin);
void render_frame(const World *world,const Game *game,int connected);
void render_set_menu_world(const World *world);
void render_set_victory_world(const World *world);
int render_set_view_animation(const void *data,size_t bytes);

/* ===== net.h ===== */
// Co-op client. Talks UDP to server/slendy_server.py through ps2ip (lwIP).
// Settings come from NET.CFG next to SLENDY.ELF (read before the IOP reset).
void net_read_config(const char *elf_path);
// Call once per frame. Joins/leaves the session as game.coop changes, sends the
// local state at ~20 Hz, applies remote players and shared custards.
void net_frame(Game *game,const World *world,float dt);

/* ===== terrain_surface.h ===== */
static inline int terrain_step(const Game *g,int menu,int victory) {
    return victory?6:menu||g->quality<2||g->time_of_day==0?4:2;
}
/* Remove one common integer repeat from the entire quad before clipping/STQ.
 * Large world-space UVs lose GS interpolation precision on real hardware.
 * Wrapping each vertex separately would change the scale or create seams. */
static inline void terrain_uv_quad(float uv[4][2],int x,int z,int step,float sx,float sz,float repeat) {
    float u=x*sx/repeat,v=-z*sz/repeat,du=step*sx/repeat,dv=step*sz/repeat;
    u-=floorf(u);v-=floorf(v-dv);
    uv[0][0]=uv[3][0]=u;uv[1][0]=uv[2][0]=u+du;
    uv[0][1]=uv[1][1]=v;uv[2][1]=uv[3][1]=v-dv;
}
/* Detect paint hidden between coarse vertices, including narrow paths. */
static inline int terrain_paint_step(const World *w,int x,int z,int step) {
    int width=w->header->width-1,error=0;
    if(x+step>=width||z+step>=width)return step;
    const uint8_t *corners[4]={w->splat+(z*width+x)*4,w->splat+(z*width+x+step)*4,
        w->splat+((z+step)*width+x+step)*4,w->splat+((z+step)*width+x)*4};
    for(int v=0;v<=step;v++)for(int u=0;u<=step;u++)for(int layer=0;layer<4;layer++) {
        int expected=u>=v?corners[0][layer]*(step-u)+corners[1][layer]*(u-v)+corners[2][layer]*v:
            corners[0][layer]*(step-v)+corners[2][layer]*u+corners[3][layer]*(v-u);
        int delta=abs((int)w->splat[((z+v)*width+x+u)*4+layer]*step-expected);
        if(delta>error)error=delta;
    }
    return error>48*step?1:error>12*step&&step>2?2:step;
}
/* Height on the two rendered triangles, including their shared 0-to-2 diagonal. */
static inline float terrain_surface_height(const World *w,int step,float x,float z) {
    float gx=x/w->config[C_SCALE],gz=z/w->config[C_SCALE+2];
    int ix=(int)floorf(gx/step)*step,iz=(int)floorf(gz/step)*step;
    if(ix<0||iz<0||ix+step>=(int)w->header->width||iz+step>=(int)w->header->height)return world_height(w,x,z);
    float u=(gx-ix)/step,v=(gz-iz)/step;
    const uint16_t *p=w->height+iz*w->header->width+ix;
    float h0=p[0],h1=p[step],h2=p[step*w->header->width+step],h3=p[step*w->header->width];
    return (u>=v?h0*(1-u)+h1*(u-v)+h2*v:h0*(1-v)+h2*u+h3*(v-u))*w->config[C_SCALE+1];
}

/* ===== transform.h ===== */
/* Camera transform uses VU0 on PS2; the host retains a scalar reference. */
static void transform_vertices(float (*output)[4],const Vertex *vertices,unsigned count,const float *matrix) {
#ifdef _EE
    if(!count)return;
    __asm__ volatile(
        "lqc2 $vf4,0(%3)\n lqc2 $vf5,16(%3)\n lqc2 $vf6,32(%3)\n lqc2 $vf7,48(%3)\n"
        "1:\n lqc2 $vf8,0(%1)\n"
        "vmulaw $ACC,$vf7,$vf0\n vmaddax $ACC,$vf4,$vf8\n vmadday $ACC,$vf5,$vf8\n vmaddz $vf9,$vf6,$vf8\n"
        "vmul.xyz $vf10,$vf9,$vf9\n vaddy.x $vf10,$vf10,$vf10\n vaddz.x $vf10,$vf10,$vf10\n vmulx.w $vf9,$vf0,$vf10\n"
        "sqc2 $vf9,0(%0)\n addiu %0,%0,16\n addiu %1,%1,32\n addiu %2,%2,-1\n bgtz %2,1b\n nop\n"
        : "+&r"(output),"+&r"(vertices),"+&r"(count):"r"(matrix):"memory");
#else
    for(unsigned i=0;i<count;i++) {
        for(int k=0;k<3;k++)output[i][k]=matrix[k]*vertices[i].x+matrix[4+k]*vertices[i].y+matrix[8+k]*vertices[i].z+matrix[12+k];
        output[i][3]=output[i][0]*output[i][0]+output[i][1]*output[i][1]+output[i][2]*output[i][2];
    }
#endif
}

/* ===== generated / test headers (not merged) ===== */
#include "../generated/fidelity.h"
#include "../generated/noonoo.h"
#include "../generated/menu/menu_layout.h"
#include "../generated/scene_visuals.h"
#include "../generated/victory/victory_layout.h"
#include "../generated/player_walk.h"
#include "../tests/menu_route.h"
#include "../tests/benchmark.h"
#include "../generated/scene_audio.h"
#include "../tests/walk_route.h"

/* ===== game.c ===== */
static float clamp(float x,float a,float b) { return x<a?a:x>b?b:x; }
static float length2(float x,float z) { return sqrtf(x*x+z*z); }
static Vec3 vec(const float *p) { Vec3 v={p[0],p[1],p[2]}; return v; }
static int span(size_t size,uint32_t off,size_t count,size_t stride) { return off<=size && count<=(size-off)/stride; }
int world_open(World *w,const void *blob,size_t length) {
    const WorldHeader *h=(const WorldHeader *)blob;
    if(length<sizeof(*h)||h->magic!=0x53505453||h->version<1||h->version>3||h->bytes!=length) return 0;
    if(h->width<2||h->height<2||h->width>1025||h->height>1025||h->textures>128) return 0;
    if(!span(length,h->mesh_offset,h->meshes,sizeof(Mesh))||!span(length,h->texture_offset,h->textures,sizeof(Texture))||
       !span(length,h->box_offset,h->boxes,sizeof(Box))||!span(length,h->tree_offset,h->trees,sizeof(Tree))||
       !span(length,h->height_offset,(size_t)h->width*h->height,2)||!span(length,h->splat_offset,(size_t)(h->width-1)*(h->height-1),h->version>=2?4:1)||
       !span(length,h->config_offset,h->version==3?132:h->version==2?130:127,4)||!span(length,h->sound_offset,h->sounds,sizeof(Sound))||
       !span(length,h->collision_offset,h->collision_meshes,sizeof(CollisionMesh))) return 0;
    memset(w,0,sizeof(*w)); w->blob=blob; w->header=h;
    w->mesh=(const Mesh *)(w->blob+h->mesh_offset); w->texture=(const Texture *)(w->blob+h->texture_offset);
    w->box=(const Box *)(w->blob+h->box_offset); w->tree=(const Tree *)(w->blob+h->tree_offset);
    w->height=(const uint16_t *)(w->blob+h->height_offset); w->splat=w->blob+h->splat_offset;
    w->config=(const float *)(w->blob+h->config_offset); w->sound=(const Sound *)(w->blob+h->sound_offset);
    w->collision=(const CollisionMesh *)(w->blob+h->collision_offset);
    for(unsigned i=0;i<h->trees;i++)if(w->tree[i].type<0||w->tree[i].type>=3||!isfinite(w->tree[i].type)||
        !isfinite(w->tree[i].x)||!isfinite(w->tree[i].z)||w->tree[i].width<=0||w->tree[i].width>2)return 0;
    if(h->version>=2) {
        unsigned index=(unsigned)w->config[127],samples=(unsigned)w->config[128];
        unsigned grid=h->version==3?(unsigned)w->config[130]:1024;
        if(grid<8||grid>1024||grid%8||!span(length,index,(grid/8)*(grid/8),2)||w->config[129]<0||w->config[129]>=h->textures)return 0;
        const uint16_t *patches=(const uint16_t *)(w->blob+index);
        for(unsigned i=0;i<(grid/8)*(grid/8);i++)if(!span(length,samples,(size_t)patches[i]+1,64))return 0;
    }
    for(uint32_t i=0;i<h->meshes;i++) {
        const Mesh *m=w->mesh+i;
        if(m->vertices>65535||m->texture>=h->textures||!span(length,m->vertex_offset,m->vertices,sizeof(Vertex))||!span(length,m->index_offset,m->triangles*3,2)) return 0;
        const uint16_t *ix=(const uint16_t *)(w->blob+m->index_offset);
        for(uint32_t j=0;j<m->triangles*3;j++) if(ix[j]>=m->vertices) return 0;
    }
    for(uint32_t i=0;i<h->textures;i++) if(!span(length,w->texture[i].offset,w->texture[i].bytes,1)) return 0;
    for(uint32_t i=0;i<h->sounds;i++) if(!span(length,w->sound[i].offset,w->sound[i].samples&AUDIO_COUNT,w->sound[i].samples&AUDIO_ULAW?1:2)) return 0;
    return 1;
}
float world_height(const World *w,float x,float z) {
    x=clamp(x/w->config[C_SCALE],0,(float)w->header->width-1.001f);
    z=clamp(z/w->config[C_SCALE+2],0,(float)w->header->height-1.001f);
    int ix=(int)x,iz=(int)z; float tx=x-ix,tz=z-iz;
    const uint16_t *p=w->height+iz*w->header->width+ix;
    return ((p[0]*(1-tx)+p[1]*tx)*(1-tz)+(p[w->header->width]*(1-tx)+p[w->header->width+1]*tx)*tz)*w->config[C_SCALE+1];
}
void game_enemy_matrix(const World *w,const Game *g,float matrix[16]) {
    float dx=g->feet.x-g->enemy.x,dy=g->feet.y-g->enemy.y,dz=g->feet.z-g->enemy.z;
    float yaw=atan2f(dx,dz),pitch=atan2f(dy,fmaxf(.00001f,length2(dx,dz)));
    float sy=sinf(yaw),cy=cosf(yaw),sp=sinf(pitch),cp=cosf(pitch);
    const float rotation[16]={cy,0,-sy,0,-sy*sp,cp,-cy*sp,0,sy*cp,sp,cy*cp,0,0,0,0,1};
    memcpy(matrix,rotation,sizeof(rotation));
    const float *origin=w->config+C_ENEMY;
    for(int row=0;row<3;row++)matrix[12+row]=(&g->enemy.x)[row]-matrix[row]*origin[0]-matrix[4+row]*origin[1]-matrix[8+row]*origin[2];
}
void game_hoover_matrix(const World *w,const Game *g,float matrix[16]) {
    steering_matrix(matrix,&g->hoover,w->config+C_HOOVER,source_hoover_rotation);
}
Vec3 game_mesh_vertex(const World *w,const Game *g,const Mesh *m,const Vertex *v) {
    Vec3 p={v->x,v->y,v->z};
    if(m->flags&(M_ENEMY|M_HOOVER)) {
        float matrix[16];if(m->flags&M_ENEMY)game_enemy_matrix(w,g,matrix);else game_hoover_matrix(w,g,matrix);
        p=(Vec3){matrix[0]*v->x+matrix[4]*v->y+matrix[8]*v->z+matrix[12],
                 matrix[1]*v->x+matrix[5]*v->y+matrix[9]*v->z+matrix[13],
                 matrix[2]*v->x+matrix[6]*v->y+matrix[10]*v->z+matrix[14]};
    }
    if(m->flags&M_DOOR) {
        const float *d=w->config+101+m->id*12;
        Vec3 end={d[0]*p.x+d[1]*p.y+d[2]*p.z+d[3],d[4]*p.x+d[5]*p.y+d[6]*p.z+d[7],d[8]*p.x+d[9]*p.y+d[10]*p.z+d[11]};
        p.x+=(end.x-p.x)*g->door_open;p.y+=(end.y-p.y)*g->door_open;p.z+=(end.z-p.z)*g->door_open;
    }
    return p;
}
static Vec3 subtract(Vec3 a,Vec3 b) { Vec3 r={a.x-b.x,a.y-b.y,a.z-b.z}; return r; }
static Vec3 cross(Vec3 a,Vec3 b) { Vec3 r={a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; return r; }
static float dot(Vec3 a,Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
static void enemy_capsule(const World *w,const Game *g,Vec3 *a,Vec3 *b) {
    float matrix[16];game_enemy_matrix(w,g,matrix);
    float half=source_enemy_capsule[4]*.5f-source_enemy_capsule[3];
    float center[3];for(int k=0;k<3;k++)center[k]=(&g->enemy.x)[k]+matrix[k]*source_enemy_capsule[0]+matrix[4+k]*source_enemy_capsule[1]+matrix[8+k]*source_enemy_capsule[2];
    for(int k=0;k<3;k++){(&a->x)[k]=center[k]-matrix[4+k]*half;(&b->x)[k]=center[k]+matrix[4+k]*half;}
}
static float segment_distance2(Vec3 p,Vec3 q,Vec3 a,Vec3 b) {
    Vec3 u=subtract(q,p),v=subtract(b,a),r=subtract(p,a);
    float uu=dot(u,u),vv=dot(v,v),uv=dot(u,v),ur=dot(u,r),vr=dot(v,r);
    float den=uu*vv-uv*uv,s=den>.000001f?clamp((uv*vr-vv*ur)/den,0,1):0;
    float t=(uv*s+vr)/fmaxf(vv,.000001f);
    if(t<0){t=0;s=clamp(-ur/fmaxf(uu,.000001f),0,1);}else if(t>1){t=1;s=clamp((uv-ur)/fmaxf(uu,.000001f),0,1);}
    Vec3 d={r.x+s*u.x-t*v.x,r.y+s*u.y-t*v.y,r.z+s*u.z-t*v.z};return dot(d,d);
}
static float enemy_support(const World *w,const Game *g,float x,float z,float oldy) {
    Vec3 a,b;enemy_capsule(w,g,&a,&b);float radius=source_enemy_capsule[3],top=-1e9f;
    for(int i=0;i<2;i++){Vec3 p=i?b:a;float rr=radius*radius-(x-p.x)*(x-p.x)-(z-p.z)*(z-p.z);
        if(rr>=0)top=fmaxf(top,p.y+sqrtf(rr));}
    Vec3 d=subtract(b,a);float dx=x-a.x,dz=z-a.z,l2=dot(d,d),horizontal=d.x*d.x+d.z*d.z,k=dx*d.x+dz*d.z;
    float constant=l2*(dx*dx+dz*dz-radius*radius)-k*k,disc=k*k*d.y*d.y-horizontal*constant;
    if(horizontal>.000001f&&disc>=0){float y=(k*d.y+sqrtf(disc))/horizontal,t=(k+y*d.y)/l2;
        if(t>=0&&t<=1)top=fmaxf(top,a.y+y);}
    return top<=oldy+.4f?top:-1e9f;
}
static int segment_triangle(Vec3 a,Vec3 b,Vec3 v0,Vec3 v1,Vec3 v2) {
    Vec3 dir=subtract(b,a),e1=subtract(v1,v0),e2=subtract(v2,v0),p=cross(dir,e2);
    float det=dot(e1,p); if(fabsf(det)<1e-6f) return 0;
    float inv=1/det; Vec3 t=subtract(a,v0); float u=dot(t,p)*inv;
    if(u<0||u>1) return 0;
    Vec3 q=cross(t,e1); float v=dot(dir,q)*inv;
    if(v<0||u+v>1) return 0;
    float hit=dot(e2,q)*inv;
    return hit>.0001f&&hit<.9999f;
}
static int near_bounds(float x,float z,const float *min,const float *max,float margin) {
    return x>=min[0]-margin&&x<=max[0]+margin&&z>=min[2]-margin&&z<=max[2]+margin;
}
/* Static broad phase. Queries retain the original triangle tests; unsupported
 * sizes fall back to the complete scan rather than dropping collision. */
static const uint16_t *collision_candidates(const World *w,float minx,float minz,float maxx,float maxz,unsigned *count) {
    *count=w->header->meshes;
#ifdef STPS_LINEAR_COLLISION
    (void)minx;(void)minz;(void)maxx;(void)maxz;return NULL;
#else
    static const Mesh *indexed;static const unsigned char *indexed_blob;static unsigned indexed_count;
    static uint16_t heads[1024],visited[2048],candidates[2048],serial;
    static struct {uint16_t mesh,next;} links[4096];static int valid;
    if(indexed!=w->mesh||indexed_blob!=w->blob||indexed_count!=w->header->meshes) {
        indexed=w->mesh;indexed_blob=w->blob;indexed_count=w->header->meshes;valid=0;
        if(indexed_count>2048)return NULL;
        memset(heads,255,sizeof(heads));unsigned used=0;
        for(unsigned i=0;i<indexed_count;i++) {
            const Mesh *m=w->mesh+i;if(!(m->flags&M_COLLIDE)||(m->flags&(M_ENEMY|M_HOOVER)))continue;
            int x0=(int)clamp(floorf(m->min[0]/32),0,31),x1=(int)clamp(floorf(m->max[0]/32),0,31);
            int z0=(int)clamp(floorf(m->min[2]/32),0,31),z1=(int)clamp(floorf(m->max[2]/32),0,31);
            for(int z=z0;z<=z1;z++)for(int x=x0;x<=x1;x++) {
                if(used>=4096)return NULL;
                unsigned cell=z*32+x;links[used].mesh=i;links[used].next=heads[cell];heads[cell]=used++;
            }
        }
        valid=1;
    }
    if(!valid)return NULL;
    if(++serial==0){memset(visited,0,sizeof(visited));serial=1;}
    int x0=(int)clamp(floorf(minx/32),0,31),x1=(int)clamp(floorf(maxx/32),0,31);
    int z0=(int)clamp(floorf(minz/32),0,31),z1=(int)clamp(floorf(maxz/32),0,31);*count=0;
    for(int z=z0;z<=z1;z++)for(int x=x0;x<=x1;x++)for(unsigned link=heads[z*32+x];link!=65535;link=links[link].next) {
        unsigned i=links[link].mesh;if(visited[i]==serial)continue;
        visited[i]=serial;candidates[(*count)++]=i;
    }
    return candidates;
#endif
}
static float enemy_ground(const World *w,float x,float z) {
    static const Mesh *meshes;static const float *config;static float offset;
    if(meshes!=w->mesh||config!=w->config) {
        float bottom=1e10f;for(uint32_t i=0;i<w->header->meshes;i++)if(w->mesh[i].flags&M_ENEMY)bottom=fminf(bottom,w->mesh[i].min[1]);
        meshes=w->mesh;config=w->config;offset=bottom<1e9f?w->config[C_ENEMY+1]-bottom:0;
    }
    return world_height(w,x,z)+offset;
}
static int box_segment(const Box *box,Vec3 a,Vec3 b) {
    float start=0,end=1; const float *aa=&a.x,*bb=&b.x;
    for(int i=0;i<3;i++) {
        float d=bb[i]-aa[i];
        if(fabsf(d)<1e-6f) { if(aa[i]<box->min[i]||aa[i]>box->max[i]) return 0; }
        else { float t1=(box->min[i]-aa[i])/d,t2=(box->max[i]-aa[i])/d;
            if(t1>t2) { float t=t1;t1=t2;t2=t; } if(t1>start) start=t1; if(t2<end) end=t2; if(start>end) return 0; }
    }
    return end>.001f&&start<.999f;
}
static int tree_collision(const World *w,Vec3 a,Vec3 b,float radius,float eye) {
    static const Tree *indexed;static uint16_t heads[64*64],next[4096];
    if(w->header->trees>4096)return 0;
    if(indexed!=w->tree) {
        memset(heads,255,sizeof(heads));
        for(unsigned i=0;i<w->header->trees;i++) {
            const Tree *t=w->tree+i;const float *c=source_tree_capsules[(int)t->type];
            int x=(int)clamp((t->x+c[0]*t->width)/16,0,63),z=(int)clamp((t->z+c[2]*t->width)/16,0,63);
            next[i]=heads[z*64+x];heads[z*64+x]=i;
        }
        indexed=w->tree;
    }
    int minx=(int)clamp((fminf(a.x,b.x)-3)/16,0,63),maxx=(int)clamp((fmaxf(a.x,b.x)+3)/16,0,63);
    int minz=(int)clamp((fminf(a.z,b.z)-3)/16,0,63),maxz=(int)clamp((fmaxf(a.z,b.z)+3)/16,0,63);
    float vx=b.x-a.x,vz=b.z-a.z,length=vx*vx+vz*vz;
    for(int z=minz;z<=maxz;z++)for(int x=minx;x<=maxx;x++)for(unsigned i=heads[z*64+x];i!=65535;i=next[i]) {
        const Tree *t=w->tree+i;const float *c=source_tree_capsules[(int)t->type];
        float tx=t->x+c[0]*t->width,tz=t->z+c[2]*t->width,r=c[3]*t->width+radius;
        float fraction=radius>0?1:clamp(((tx-a.x)*vx+(tz-a.z)*vz)/fmaxf(length,.0001f),0,1);
        float dx=a.x+vx*fraction-tx,dz=a.z+vz*fraction-tz;
        if(dx*dx+dz*dz>=r*r)continue;
        float y=a.y+(b.y-a.y)*fraction,bottom=t->y+(c[1]-c[4]*.5f)*t->height,top=bottom+c[4]*t->height;
        if(y+eye<bottom||y+radius>top)continue;
        float ox=a.x-tx,oz=a.z-tz;
        if(radius==0||dx*dx+dz*dz<ox*ox+oz*oz)return 1;
    }
    return 0;
}
int world_occluded(const World *w,const Game *g,Vec3 a,Vec3 b) {
    if(tree_collision(w,a,b,0,0))return 1;
    for(uint32_t i=0;i<w->header->boxes;i++) if(box_segment(w->box+i,a,b)) return 1;
    Box ray={{fminf(a.x,b.x),fminf(a.y,b.y),fminf(a.z,b.z)},{fmaxf(a.x,b.x),fmaxf(a.y,b.y),fmaxf(a.z,b.z)}};
    unsigned count;const uint16_t *candidates=collision_candidates(w,ray.min[0],ray.min[2],ray.max[0],ray.max[2],&count);
    for(unsigned candidate=0;candidate<count;candidate++) {unsigned i=candidates?candidates[candidate]:candidate;
        const Mesh *m=w->mesh+i; if(!(m->flags&M_COLLIDE)||(m->flags&(M_ENEMY|M_HOOVER))||((m->flags&M_DAY_TREE)&&g->time_of_day==2)||((m->flags&M_DOOR)&&g->door_open>.7f)) continue;
        if(m->max[0]<ray.min[0]||m->min[0]>ray.max[0]||m->max[2]<ray.min[2]||m->min[2]>ray.max[2]||m->max[1]<ray.min[1]||m->min[1]>ray.max[1]) continue;
        const Vertex *v=(const Vertex *)(w->blob+m->vertex_offset); const uint16_t *ix=(const uint16_t *)(w->blob+m->index_offset);
        for(uint32_t j=0;j<m->triangles;j++,ix+=3) {
            Vec3 p0={v[ix[0]].x,v[ix[0]].y,v[ix[0]].z},p1={v[ix[1]].x,v[ix[1]].y,v[ix[1]].z},p2={v[ix[2]].x,v[ix[2]].y,v[ix[2]].z};
            if(segment_triangle(a,b,p0,p1,p2)) return 1;
        }
    }
    return 0;
}
static float ground(const World *w,const Game *g,float x,float z,float oldy) {
    float result=world_height(w,x,z);
    if(g->state==ST_PLAY&&(x-g->enemy.x)*(x-g->enemy.x)+(z-g->enemy.z)*(z-g->enemy.z)<16)result=fmaxf(result,enemy_support(w,g,x,z,oldy));
    // Close the upper opening using the original rim's centre, radius and height.
    // It supports arrivals from the roof without interfering with the room below.
    float roof_x=x-source_roof_support[0],roof_z=z-source_roof_support[1];
    if(roof_x*roof_x+roof_z*roof_z<source_roof_support[2]*source_roof_support[2]&&source_roof_support[3]<=oldy+.4f)
        result=fmaxf(result,source_roof_support[3]);
    for(unsigned i=0;i<w->header->boxes;i++) {
        const Box *b=w->box+i;
        if(near_bounds(x,z,b->min,b->max,0)&&b->max[1]>result&&b->max[1]<=oldy+.4f)result=b->max[1];
    }
    unsigned count;const uint16_t *candidates=collision_candidates(w,x-.4f,z-.4f,x+.4f,z+.4f,&count);
    for(unsigned candidate=0;candidate<count;candidate++) {unsigned i=candidates?candidates[candidate]:candidate;
        const Mesh *m=w->mesh+i; if(!(m->flags&M_COLLIDE)||(m->flags&(M_ENEMY|M_HOOVER))||((m->flags&M_DAY_TREE)&&g->time_of_day==2)||!near_bounds(x,z,m->min,m->max,.4f)) continue;
        const Vertex *v=(const Vertex *)(w->blob+m->vertex_offset); const uint16_t *ix=(const uint16_t *)(w->blob+m->index_offset);
        for(uint32_t j=0;j<m->triangles;j++,ix+=3) {
            const Vertex *a=v+ix[0],*b=v+ix[1],*c=v+ix[2];
            Vec3 e1={b->x-a->x,b->y-a->y,b->z-a->z},e2={c->x-a->x,c->y-a->y,c->z-a->z}; Vec3 n=cross(e1,e2);
            float length=sqrtf(dot(n,n));if(length<.000001f||fabsf(n.y)<length*.001f)continue;
            float contact_x=x,contact_z=z;
            float den=(b->z-c->z)*(a->x-c->x)+(c->x-b->x)*(a->z-c->z); if(fabsf(den)<.00001f) continue;
            float u=((b->z-c->z)*(contact_x-c->x)+(c->x-b->x)*(contact_z-c->z))/den;
            float t=((c->z-a->z)*(contact_x-c->x)+(a->x-c->x)*(contact_z-c->z))/den;
            // World coordinates are large; shared edges need a small barycentric tolerance.
            if(u<-.0001f||t<-.0001f||u+t>1.0001f) continue;
            float y=a->y*u+b->y*t+c->y*(1-u-t);
            // A steep roof still collides with a descending controller. Slope
            // limits must never remove the triangle from vertical collision.
            if(y>result&&y<=oldy+.4f) result=y;
        }
    }
    (void)g; return result;
}
static int blocked(const World *w,const Game *g,Vec3 a,Vec3 b) {
    if((g->state==ST_PLAY||g->state==ST_PAUSE)&&(b.x-g->enemy.x)*(b.x-g->enemy.x)+(b.z-g->enemy.z)*(b.z-g->enemy.z)<16) {
        Vec3 ea,eb;enemy_capsule(w,g,&ea,&eb);float r=.4f+source_enemy_capsule[3];
        Vec3 bottom=b,top=b;bottom.y+=.4f;top.y+=g->eye-.1f;
        float next=segment_distance2(bottom,top,ea,eb);bottom=a;top=a;bottom.y+=.4f;top.y+=g->eye-.1f;
        if(next<r*r&&next<segment_distance2(bottom,top,ea,eb))return 1;
    }
    if(tree_collision(w,a,b,.4f,g->eye))return 1;
    for(uint32_t i=0;i<w->header->boxes;i++) {
        const Box *box=w->box+i;
        if(near_bounds(b.x,b.z,box->min,box->max,.4f)&&b.y+g->eye>box->min[1]&&b.y+.4f<box->max[1]) return 1;
    }
    for(int j=0;j<3;j++) {
        Vec3 from=a,to=b; from.y+=.45f+j*g->eye*.4f; to.y=from.y;
        float dx=b.x-a.x,dz=b.z-a.z,d=length2(dx,dz);
        if(d>.00001f) { to.x+=dx/d*.4f;to.z+=dz/d*.4f; }
        if(world_occluded(w,g,from,to)) return 1;
    }
    return world_height(w,b.x,b.z)>a.y+.4f;
}
void game_init(Game *g) { memset(g,0,sizeof(*g));g->state=ST_MENU;g->time_of_day=2;g->flashlight=1;g->quality=3;g->draw_grass=1; }
static uint32_t random_number(Game *g) {g->random=g->random*1664525u+1013904223u;return g->random;}
void game_start(Game *g,const World *w) {
    int day=g->time_of_day,quality=g->quality;uint32_t seed=0x53545053u^(uint32_t)(g->state_time*1000000);
    memset(g,0,sizeof(*g)); g->time_of_day=day;g->quality=quality;g->random=seed;
    g->state=ST_PLAY;g->flashlight=1;g->health=100;g->eye=1.7f;g->draw_grass=1;
    g->feet=vec(w->config+C_SPAWN);g->enemy=vec(w->config+C_ENEMY);
    steering_init(&g->hoover,w->config+C_HOOVER,source_hoover_rotation);
    g->feet.y=world_height(w,g->feet.x,g->feet.z);g->enemy.y=enemy_ground(w,g->enemy.x,g->enemy.z);
    g->enemy_speed=w->config[C_CHASE];g->grounded=1;g->next_scare=32+random_number(g)%170;
}
// Apply custards collected by teammates (shared objective in co-op).
void game_coop_merge(Game *g,const World *w,uint16_t mask) {
    uint16_t fresh=mask&0x3FF&~g->custards;
    if(!fresh||g->state!=ST_PLAY)return;
    g->custards|=fresh;
    int n=0;for(int i=0;i<10;i++)if(g->custards&(1<<i))n++;
    g->collected=n;g->events|=EV_PICKUP;g->pickup_time=3;
    if(10-n<=5)g->enemy_speed=w->config[C_FAST];
    if(10-n<=2)g->enemy_speed=w->config[C_SPRINT];
    if(n>=10){g->state=ST_WON;g->state_time=0;}
}
static void update_menu(Game *g,const World *w,const Input *in) {
    const int count[6]={5,7,4,3,4,4};int n=count[g->menu_page];
    if(in->pressed&(B_UP|B_LEFT)) g->menu=(g->menu+n-1)%n;
    if(in->pressed&(B_DOWN|B_RIGHT)) g->menu=(g->menu+1)%n;
    if(in->pressed&B_BACK) {g->menu_page=MENU_MAIN;g->menu=0;return;}
    if(in->pressed&B_CONFIRM) {
        if(g->menu_page==MENU_MAIN) {
            if(g->menu==0)g->menu_page=MENU_SINGLE;
            else if(g->menu==1)g->menu_page=MENU_MULTI;
            else if(g->menu==2)g->menu_page=MENU_SETTINGS;
            else if(g->menu==3)g->state=ST_CREDITS;
            else g->state=ST_HELP;
            g->menu=0;
        } else if(g->menu_page==MENU_SETTINGS) {
            if(g->menu<6)g->quality=g->menu;
            else {g->menu_page=MENU_MAIN;g->menu=0;}
        } else if(g->menu_page==MENU_MULTI) {
            g->menu_page=g->menu==0?MENU_COOP:g->menu==1?MENU_VERSUS:MENU_MAIN;g->menu=0;
        } else if(g->menu==3) {g->menu_page=MENU_MAIN;g->menu=0;}
        else {
            g->time_of_day=g->menu;
            if(g->menu_page==MENU_SINGLE)game_start(g,w);
            else if(g->menu_page==MENU_COOP){game_start(g,w);g->coop=1;}
            else g->state=ST_NETWORK;
        }
    }
}
static void player_motion(Game *g,const World *w,const Input *in,float dt) {
    g->eye+=((g->crouching?.85f:1.7f)-g->eye)*fminf(1,dt*5);
    g->yaw+=in->look_x*2.3f*dt;g->pitch=clamp(g->pitch-in->look_y*1.8f*dt,-1.0472f,1.0472f);
    float speed=w->config[g->crouching?C_CROUCH:((in->held&B_RUN)?C_RUN:C_WALK)];
    float mx=in->move_x,my=in->move_y,mag=length2(mx,my);if(mag>1){mx/=mag;my/=mag;}
    g->moving=mag>.1f;
    float dx=(mx*cosf(g->yaw)+my*sinf(g->yaw))*speed*dt,dz=(-mx*sinf(g->yaw)+my*cosf(g->yaw))*speed*dt;
    float old_ground=g->feet.y;int standing=g->grounded&&!g->moving;Vec3 next=g->feet;
    if(fabsf(dx)>.00001f){next.x+=dx;if(!blocked(w,g,g->feet,next))g->feet.x=next.x;}
    if(fabsf(dz)>.00001f){next=g->feet;next.z+=dz;if(!blocked(w,g,g->feet,next))g->feet.z=next.z;}
    g->feet.x=clamp(g->feet.x,.4f,(w->header->width-1)*w->config[C_SCALE]-.4f);
    g->feet.z=clamp(g->feet.z,.4f,(w->header->height-1)*w->config[C_SCALE+2]-.4f);
    if((in->pressed&B_JUMP)&&g->grounded&&!g->crouching){g->vy=sqrtf(30);g->grounded=0;}
    g->vy=fmaxf(-20,g->vy-15*dt);g->feet.y+=g->vy*dt;
    // Test from the pre-integration height, so fast descent cannot tunnel past a floor.
    float gy=standing&&length2(g->feet.x-g->enemy.x,g->feet.z-g->enemy.z)>4?old_ground:ground(w,g,g->feet.x,g->feet.z,old_ground);
    if(g->feet.y<=gy){g->feet.y=gy;g->vy=0;g->grounded=1;}else g->grounded=0;
}
static int enemy_side_spot(const World *w,const Game *g,Vec3 feet) {
    if(feet.x<1||feet.z<1||feet.x>(w->header->width-1)*w->config[C_SCALE]-1||feet.z>(w->header->height-1)*w->config[C_SCALE+2]-1)return 0;
    if(fabsf(feet.y-g->feet.y)>1)return 0;
    Vec3 approach=feet;approach.x+=3;
    if(tree_collision(w,approach,feet,.55f,2.3f))return 0;
    for(unsigned i=0;i<w->header->boxes;i++) {
        const Box *box=w->box+i;
        if(near_bounds(feet.x,feet.z,box->min,box->max,.55f)&&feet.y+2.3f>box->min[1]&&feet.y+.35f<box->max[1])return 0;
    }
    Vec3 view=g->feet;view.y+=g->eye;Vec3 target=feet;target.y+=1.2f;
    if(world_occluded(w,g,view,target))return 0;
    Vec3 a=target,b=target;a.x-=.55f;b.x+=.55f;
    if(world_occluded(w,g,a,b))return 0;
    a=target;b=target;a.z-=.55f;b.z+=.55f;
    return !world_occluded(w,g,a,b);
}
static int enemy_shift_side(Game *g,const World *w) {
    float side=(g->enemy.x-g->feet.x)*cosf(g->yaw)-(g->enemy.z-g->feet.z)*sinf(g->yaw)>0?-1:1;
    for(int attempt=0;attempt<4;attempt++) {
        float sign=attempt&1?-side:side,angle=g->yaw+sign*.55f;
        float radius=w->config[C_FOLLOW]+.35f+(attempt/2)*.75f;
        Vec3 spot={g->feet.x+sinf(angle)*radius,0,g->feet.z+cosf(angle)*radius};
        spot.y=world_height(w,spot.x,spot.z);
        if(!enemy_side_spot(w,g,spot))continue;
        spot.y=enemy_ground(w,spot.x,spot.z);g->enemy=spot;g->next_enemy_shift=g->time+.25f;
        g->events|=EV_ENEMY_SHIFT;return 1;
    }
    g->next_enemy_shift=g->time+.1f;return 0;
}
void game_update(Game *g,const World *w,const Input *in,float dt) {
    g->events=0;dt=clamp(dt,0,.05f);g->state_time+=dt;
    if(g->state==ST_MENU) { update_menu(g,w,in); return; }
    if(g->state==ST_HELP||g->state==ST_CREDITS) { if(in->pressed&(B_CONFIRM|B_BACK)) g->state=ST_MENU; return; }
    if(g->state==ST_NETWORK) {if(in->pressed&B_BACK)g->state=ST_MENU;return;}
    if(g->state==ST_WINSCENE) {
        if(in->pressed&(B_CONFIRM|B_BACK)){g->state=ST_MENU;g->menu_page=MENU_MAIN;g->menu=0;g->state_time=0;}
        return;
    }
    if(g->state==ST_WON) {
        player_motion(g,w,in,dt);
        if(g->state_time>=5){g->state=ST_WINSCENE;g->state_time=0;}
        return;
    }
    if(g->state==ST_DEAD) {
        if(g->state_time>5||((in->pressed&B_CONFIRM)&&g->state_time>1)) {g->state=ST_MENU;g->state_time=0;return;}
        if(g->state==ST_DEAD)player_motion(g,w,in,dt);
        return;
    }
    if(in->pressed&B_PAUSE) { g->state=g->state==ST_PLAY?ST_PAUSE:ST_PLAY;g->moving=0;return; }
    if(g->state==ST_PAUSE) { if(in->pressed&B_BACK) { g->state=ST_MENU;g->state_time=0; } return; }
    g->time+=dt;g->scare_time=fmaxf(0,g->scare_time-dt);g->pickup_time=fmaxf(0,g->pickup_time-dt);
    if(in->pressed&B_LIGHT) { g->flashlight=!g->flashlight;g->events|=EV_SWITCH; }
    if(in->pressed&B_MAP) g->map=!g->map;
    if(in->pressed&B_CROUCH) g->crouching=!g->crouching;
    player_motion(g,w,in,dt);
    for(int i=0;i<10;i++) {
        const float *p=w->config+C_CUSTARD+i*3;
        if(!(g->custards&(1<<i))&&length2(g->feet.x-p[0],g->feet.z-p[2])<1.35f&&fabsf(g->feet.y-p[1])<2.5f) {
            g->custards|=1<<i;g->collected++;g->events|=EV_PICKUP;g->pickup_time=3;
            if(10-g->collected==5) g->enemy_speed=w->config[C_FAST];
            if(10-g->collected==2) g->enemy_speed=w->config[C_SPRINT];
            if(g->collected==10) {g->state=ST_WON;g->state_time=0;return;}
        }
    }
    float ex=g->enemy.x-g->feet.x,ez=g->enemy.z-g->feet.z,ey=g->enemy.y-g->feet.y;
    float distance=sqrtf(ex*ex+ez*ez+ey*ey);
    Vec3 view=g->feet;view.y+=g->eye;Vec3 target=g->enemy;target.y+=1;
    Vec3 sight=subtract(target,view);float sight_length=sqrtf(dot(sight,sight));
    float product=sight_length>.001f?(cosf(g->pitch)*(sinf(g->yaw)*sight.x+cosf(g->yaw)*sight.z)+sinf(g->pitch)*sight.y)/sight_length:1;
    int visible=product>w->config[C_VISIBLE]&&distance<fmaxf(30,w->config[C_RANGE])&&!world_occluded(w,g,view,target);
    // User reference: damage and lateral reappearances belong to a close
    // encounter. A distant sighting must not teleport into damage range.
    float attack_range=fminf(w->config[C_RANGE],5);
    int close=distance<attack_range,above=g->feet.y>g->enemy.y+source_enemy_capsule[3]*.75f;
    g->enemy_attacking=visible&&close;
    if(visible) {
        if(close)g->health=fmaxf(0,g->health-w->config[C_DAMAGE]*dt);
        if(!g->enemy_visible)g->events|=EV_ENEMY_SEEN;
        if(g->health<=0) {g->state=ST_DEAD;g->state_time=0;g->events|=EV_DEATH;return;}
        if(close&&!above&&g->time>=g->next_enemy_shift)enemy_shift_side(g,w);
    } else if(g->enemy_visible&&product<w->config[C_OFFSCREEN]&&close&&!above&&g->time>=g->next_enemy_shift&&enemy_shift_side(g,w)) {
        // Recreate the lateral reappearance seen in the reference video.
    }
    if(!(g->events&EV_ENEMY_SHIFT)&&((visible&&!close)||(!visible&&(product<w->config[C_OFFSCREEN]||distance>=attack_range)))) {
        if(distance>w->config[C_FOLLOW]) {
            float step=fminf(g->enemy_speed*dt,distance-w->config[C_FOLLOW]);
            g->enemy.x-=ex/distance*step;g->enemy.z-=ez/distance*step;
            g->enemy.y=enemy_ground(w,g->enemy.x,g->enemy.z);
        }
    }
    g->enemy_visible=visible;
    if(length2(g->feet.x-w->config[C_DOOR],g->feet.z-w->config[C_DOOR+2])<6) g->door_open=fminf(1,g->door_open+dt*1.5f);
    steering_step(&g->hoover,w->config+C_WAY,source_hoover_settings[0],source_hoover_settings[1],source_hoover_settings[2],dt);
    if(g->time>g->next_scare) {g->scare_time=.1f;g->scare_image=1+random_number(g)%6;g->events|=EV_SCARE;g->next_scare+=33+random_number(g)%170;}
}
// Integrate slow display frames without slowing the game clock. Button edges
// fire only once, and audio events survive all physics steps in this frame.
void game_advance(Game *g,const World *w,const Input *in,float elapsed) {
    float remaining=clamp(elapsed,0,.25f); Input step=*in;int events=0;
    do {
        float dt=fminf(remaining,1/60.f);
        game_update(g,w,&step,dt);events|=g->events;step.pressed=0;
        remaining-=dt;
    } while(remaining>.000001f);
    g->events=events;
}

/* ===== net.c ===== */
// Slendytubbies PS2 co-op client (UDP via ps2ip/lwIP). See server/slendy_server.py
// for the wire format. Not part of the original port; MIT like the rest of the code.
extern unsigned char dev9_start[],dev9_end[],netman_start[],netman_end[];
extern unsigned char smap_start[],smap_end[],ps2ipnm_start[],ps2ipnm_end[];

enum { PK_HELLO=1,PK_WELCOME=2,PK_STATE=3,PK_PEERS=4,PK_BYE=5 };
enum { MAX_PEERS=3,SEND_HZ=20 };
typedef struct {char ip[16],mask[16],gateway[16],server[16];int port;int valid;} NetConfig;
static NetConfig cfg;
static int net_ready,net_failed,sock=-1,joined,my_id=255;
static float send_timer,silence;
typedef struct {int used;float x,y,z,yaw;} PeerTarget;
static PeerTarget target[4];

static int parse_ip(const char *s,int out[4]) {
    return sscanf(s,"%d.%d.%d.%d",out,out+1,out+2,out+3)==4;
}
// Must run before SifIopReset: it uses the launcher's mass: driver.
void net_read_config(const char *elf_path) {
    memset(&cfg,0,sizeof(cfg));strcpy(cfg.mask,"255.255.255.0");cfg.port=7777;
    if(!elf_path)return;
    char path[256];strncpy(path,elf_path,sizeof(path)-16);path[sizeof(path)-16]=0;
    char *slash=strrchr(path,'/');if(!slash)slash=strrchr(path,'\\');
    if(slash)strcpy(slash+1,"NET.CFG");
    else if(!strrchr(path,':'))strcpy(path,"NET.CFG");
    else {char *colon=strrchr(path,':');strcpy(colon+1,"NET.CFG");}
    FILE *f=fopen(path,"r");printf("STPS NET cfg %s %s\n",path,f?"found":"missing");
    if(!f)return;
    char line[96];
    while(fgets(line,sizeof(line),f)) {
        char *eq=strchr(line,'=');if(!eq||line[0]=='#')continue;*eq++=0;
        for(char *e=eq;*e;e++)if(*e=='\r'||*e=='\n'||*e==' ')*e=0;
        if(!strcmp(line,"ip"))strncpy(cfg.ip,eq,15);
        else if(!strcmp(line,"netmask"))strncpy(cfg.mask,eq,15);
        else if(!strcmp(line,"gateway"))strncpy(cfg.gateway,eq,15);
        else if(!strcmp(line,"server"))strncpy(cfg.server,eq,15);
        else if(!strcmp(line,"port"))cfg.port=atoi(eq);
    }
    fclose(f);
    int a[4];cfg.valid=parse_ip(cfg.ip,a)&&parse_ip(cfg.server,a)&&cfg.port>0&&cfg.port<65536;
    printf("STPS NET ip=%s server=%s:%d valid=%d\n",cfg.ip,cfg.server,cfg.port,cfg.valid);
}
static int load(void *start,void *end,const char *name) {
    int result=0,id=SifExecModuleBuffer(start,(unsigned char *)end-(unsigned char *)start,0,NULL,&result);
    printf("STPS NET MODULE %s id=%d result=%d\n",name,id,result);return id>=0&&result>=0;
}
static struct sockaddr_in server_addr;
static int net_start(void) {
    if(!cfg.valid)return 0;
    if(!(load(dev9_start,dev9_end,"dev9")&&load(netman_start,netman_end,"netman")&&
         load(smap_start,smap_end,"smap")&&load(ps2ipnm_start,ps2ipnm_end,"ps2ip-nm")))return 0;
    if(NetManInit()<0)return 0;
    int a[4],m[4],g[4]={0,0,0,0};
    parse_ip(cfg.ip,a);parse_ip(cfg.mask,m);if(cfg.gateway[0])parse_ip(cfg.gateway,g);
    ip4_addr_t ip,mask,gw;IP4_ADDR(&ip,a[0],a[1],a[2],a[3]);IP4_ADDR(&mask,m[0],m[1],m[2],m[3]);IP4_ADDR(&gw,g[0],g[1],g[2],g[3]);
    ps2ipInit(&ip,&mask,&gw);
    // Wait up to ~8 s for the Ethernet link (cable + adapter).
    int up=0;for(int i=0;i<80&&!up;i++) {
        up=NetManIoctl(NETMAN_NETIF_IOCTL_ETH_GET_LINK_STATUS,NULL,0,NULL,0)==NETMAN_NETIF_ETH_LINK_STATE_UP;
        if(!up)DelayThread(100*1000);
    }
    printf("STPS NET link=%d\n",up);if(!up)return 0;
    sock=socket(AF_INET,SOCK_DGRAM,0);if(sock<0)return 0;
    int s[4];parse_ip(cfg.server,s);
    memset(&server_addr,0,sizeof(server_addr));server_addr.sin_family=AF_INET;server_addr.sin_len=sizeof(server_addr);
    server_addr.sin_port=htons(cfg.port);server_addr.sin_addr.s_addr=htonl(((unsigned)s[0]<<24)|(s[1]<<16)|(s[2]<<8)|s[3]);
    return 1;
}
static void send_packet(const void *data,int length) {
    sendto(sock,data,length,0,(struct sockaddr *)&server_addr,sizeof(server_addr));
}
static void send_simple(int type) {
    unsigned char p[4]={'S','T',(unsigned char)type,(unsigned char)(my_id&255)};send_packet(p,4);
}
static void send_state(const Game *g) {
    unsigned char p[24];p[0]='S';p[1]='T';p[2]=PK_STATE;p[3]=(unsigned char)my_id;
    float v[4]={g->feet.x,g->feet.y,g->feet.z,g->yaw};memcpy(p+4,v,16);
    p[20]=(unsigned char)((g->flashlight?1:0)|(g->crouching?2:0)|(g->moving?4:0)|(g->state==ST_DEAD?8:0));p[21]=0;
    uint16_t mask=g->custards&0x3FF;memcpy(p+22,&mask,2);send_packet(p,24);
}
static void receive(Game *g,const World *w) {
    unsigned char buf[256];struct sockaddr_in from;
    for(;;) {
        socklen_t length=sizeof(from);
        int n=recvfrom(sock,buf,sizeof(buf),MSG_DONTWAIT,(struct sockaddr *)&from,&length);
        if(n<4)break;
        if(buf[0]!='S'||buf[1]!='T')continue;
        silence=0;
        if(buf[2]==PK_WELCOME)my_id=buf[3];
        else if(buf[2]==PK_PEERS&&n>=8) {
            int count=buf[4];uint16_t mask;memcpy(&mask,buf+6,2);
            if(count>MAX_PEERS||n<8+count*20)continue;
            for(int i=0;i<4;i++)target[i].used=0;
            for(int i=0;i<count;i++) {
                const unsigned char *e=buf+8+i*20;int id=e[0];if(id>3)continue;
                float v[4];memcpy(v,e+4,16);
                int fresh=!target[id].used&&!g->peer[id].active;
                target[id]=(PeerTarget){1,v[0],v[1],v[2],v[3]};
                g->peer[id].flags=e[1];g->peer[id].active=1;
                if(fresh){g->peer[id].feet=(Vec3){v[0],v[1],v[2]};g->peer[id].yaw=v[3];}
            }
            for(int i=0;i<4;i++)if(!target[i].used)g->peer[i].active=0;
            game_coop_merge(g,w,mask);
        }
    }
}
static void leave(Game *g) {
    if(joined){send_simple(PK_BYE);joined=0;}
    my_id=255;memset(g->peer,0,sizeof(g->peer));memset(target,0,sizeof(target));
}
void net_frame(Game *g,const World *w,float dt) {
    if(!g->coop){if(joined)leave(g);return;}
    // Leaving the match (menu) or dying ends the session for this player.
    if(g->state==ST_MENU||g->state==ST_DEAD||g->state==ST_WINSCENE) {leave(g);g->coop=0;return;}
    if(!net_ready&&!net_failed)net_ready=net_start();
    if(!net_ready) {net_failed=1;g->coop=0;g->state=ST_NETWORK;return;}
    if(!joined){memset(target,0,sizeof(target));send_simple(PK_HELLO);joined=1;send_timer=0;silence=0;}
    receive(g,w);
    silence+=dt;
    if(silence>6) {g->coop=0;g->state=ST_NETWORK;leave(g);return;}  // server lost
    send_timer+=dt;
    if(send_timer>=1.f/SEND_HZ) {
        send_timer=0;if(my_id==255)send_simple(PK_HELLO);else send_state(g);
    }
    float k=fminf(1,dt*12);
    for(int i=0;i<4;i++)if(g->peer[i].active&&target[i].used) {
        Vec3 *f=&g->peer[i].feet;f->x+=(target[i].x-f->x)*k;f->y+=(target[i].y-f->y)*k;f->z+=(target[i].z-f->z)*k;
        float d=target[i].yaw-g->peer[i].yaw;while(d>3.14159f)d-=6.28318f;while(d<-3.14159f)d+=6.28318f;
        g->peer[i].yaw+=d*k;
    }
}

/* ===== render.c ===== */
typedef struct { float x,y,z,u,v,r,g,b,f,alpha; unsigned mask; } CameraVertex;
enum { TRIANGLE_BUDGET=20000 };
static const World *rworld;static const Game *rgame;
static const World *menu_world,*victory_world;static int menu_scene,menu_baked,win_scene;
static const float *active_sun;
static const unsigned *active_tree_colors;
static const float (*active_grass)[10];
static const unsigned *active_grass_modes;
static const float *walk_frames;
int render_set_view_animation(const void *data,size_t bytes) {
    if(bytes!=VIEW_WALK_FRAMES*VIEW_WALK_VERTICES*6*sizeof(float))return 0;
    walk_frames=data;return 1;
}
static const float *menu_matrix;
void render_set_menu_world(const World *w) {menu_world=w;}
void render_set_victory_world(const World *w) {victory_world=w;}
static float sy,cy,sp,cp,far_distance,density,fog[3],ambient[3];
static int triangles;
static float draw_color[3]={1,1,1};
static unsigned draw_alpha=128;
static int cull_faces;
static float fog_table[4097];static float cached_density=-1;
static float clampf(float x,float min,float max) { return x<min?min:x>max?max:x; }
static int tree_in_view(const Tree *tree,const float bounds[6]);
static void prepare_vertex(CameraVertex *v) {
    v->mask=(v->z<.15f?1:0)|(v->x+v->z*.7698f<0?2:0)|(v->z*.7698f-v->x<0?4:0)
           |(v->y+v->z*.57735f<0?8:0)|(v->z*.57735f-v->y<0?16:0);
}
static CameraVertex camera(Vec3 p,float u,float v,float nx,float ny,float nz,const float *color) {
    float dx=p.x-rgame->feet.x,dy=p.y-rgame->feet.y-rgame->eye,dz=p.z-rgame->feet.z;
    float vz=dx*sy+dz*cy; CameraVertex out;
    out.x=dx*cy-dz*sy;out.y=dy*cp-vz*sp;out.z=dy*sp+vz*cp;out.u=u;out.v=v;
    if(menu_scene) {
        out.x=dx*menu_matrix[0]+dy*menu_matrix[4]+dz*menu_matrix[8];
        out.y=dx*menu_matrix[1]+dy*menu_matrix[5]+dz*menu_matrix[9];
        out.z=dx*menu_matrix[2]+dy*menu_matrix[6]+dz*menu_matrix[10];
    }
    float distance2=dx*dx+dy*dy+dz*dz,light=0;
    if(rgame->flashlight&&distance2<400) {
        float dist=sqrtf(distance2);
        float cone=out.z/fmaxf(dist,.001f);light=clampf((cone-.9063078f)*18,0,1)*clampf(1-dist/20,0,1)*2.5f;
    }
    float diffuse=clampf(ny*.5f-nx*.15f-nz*.15f+.6f,.3f,1);
    int fog_index=(int)(distance2*.125f);float visibility=fog_table[fog_index<4096?fog_index:4096];
    out.r=clampf(color[0]*(ambient[0]*diffuse+light),0,1.7f);
    out.g=clampf(color[1]*(ambient[1]*diffuse+light),0,1.7f);
    out.b=clampf(color[2]*(ambient[2]*diffuse+light),0,1.7f);
    out.f=visibility;out.alpha=1;
    if(!menu_scene) {
        const float *sun=active_sun;float lambert=fmaxf(0,nx*sun[0]+ny*sun[1]+nz*sun[2]);
        out.r=clampf(out.r+color[0]*sun[3]*lambert,0,1.99f);
        out.g=clampf(out.g+color[1]*sun[4]*lambert,0,1.99f);
        out.b=clampf(out.b+color[2]*sun[5]*lambert,0,1.99f);
    }
    if(menu_baked) {out.r=nx;out.g=ny;out.b=nz;}
    prepare_vertex(&out);return out;
}
static float (*vertex_light)[3];static unsigned *light_offsets,*tree_order;
static unsigned tree_first[15],tree_count[15],tree_cost[15];static float tree_bounds[3][6];
static size_t light_capacity,offset_capacity;static const World *lit_world;
static int lit_day=-1;
static int cache_lights(void) {
    if(lit_world==rworld&&lit_day==rgame->time_of_day)return 1;
    if(rworld->header->meshes>offset_capacity){unsigned *next=realloc(light_offsets,rworld->header->meshes*sizeof(*next));
        if(!next)return 0;
        light_offsets=next;offset_capacity=rworld->header->meshes;}
    if(lit_world!=rworld) {
        unsigned *next=realloc(tree_order,rworld->header->meshes*sizeof(*next));if(!next)return 0;tree_order=next;
        unsigned cursor=0;
        for(int kind=0;kind<15;kind++) {
            tree_first[kind]=cursor;tree_count[kind]=tree_cost[kind]=0;
            if(kind<3)for(int k=0;k<3;k++){tree_bounds[kind][k]=1e9f;tree_bounds[kind][k+3]=-1e9f;}
            for(unsigned i=0;i<rworld->header->meshes;i++) {
                const Mesh *m=rworld->mesh+i;if(!(m->flags&M_TREE)||m->id!=(unsigned)kind)continue;
                tree_order[cursor++]=i;tree_count[kind]++;tree_cost[kind]+=m->triangles;
                if(kind<3)for(int k=0;k<3;k++){tree_bounds[kind][k]=fminf(tree_bounds[kind][k],m->min[k]);tree_bounds[kind][k+3]=fmaxf(tree_bounds[kind][k+3],m->max[k]);}
            }
        }
    }
    unsigned total=0;
    for(unsigned i=0;i<rworld->header->meshes;i++){light_offsets[i]=total;total+=rworld->mesh[i].vertices;}
    if(total>light_capacity){void *next=realloc(vertex_light,total*sizeof(*vertex_light));if(!next)return 0;vertex_light=next;light_capacity=total;}
    const float *sun=active_sun;
    for(unsigned i=0;i<rworld->header->meshes;i++) {
        const Mesh *m=rworld->mesh+i;const Vertex *v=(const Vertex *)(rworld->blob+m->vertex_offset);
        for(unsigned j=0;j<m->vertices;j++) {
            float diffuse=clampf(v[j].ny*.5f-v[j].nx*.15f-v[j].nz*.15f+.6f,.3f,1);
            float lambert=fmaxf(0,v[j].nx*sun[0]+v[j].ny*sun[1]+v[j].nz*sun[2]);
            for(int k=0;k<3;k++)vertex_light[light_offsets[i]+j][k]=m->color[k]*(ambient[k]*diffuse+sun[3+k]*lambert);
        }
    }
    lit_world=rworld;lit_day=rgame->time_of_day;return 1;
}
static CameraVertex camera_view(const float *position,float u,float v) {
    CameraVertex out;out.x=position[0];out.y=position[1];out.z=position[2];out.u=u;out.v=v;
    out.r=position[3];out.g=out.b=0;out.f=1;out.alpha=1;
    prepare_vertex(&out);return out;
}
static float view_matrix[16] __attribute__((aligned(16)));
static float transformed[4096][4] __attribute__((aligned(16)));
static void camera_vertices(CameraVertex *out,const Vertex *vertices,unsigned count,const float *pose) {
    for(unsigned offset=0;offset<count;offset+=4096) {
        unsigned batch=count-offset;if(batch>4096)batch=4096;
        transform_vertices(transformed,vertices+offset,batch,pose);
        for(unsigned j=0;j<batch;j++) {
            out[offset+j]=camera_view(transformed[j],vertices[offset+j].u,vertices[offset+j].v);
            out[offset+j].mask|=(offset+j+1)<<8;
        }
    }
}
static void prepare_view_matrix(void) {
    float x=-rgame->feet.x,y=-rgame->feet.y-rgame->eye,z=-rgame->feet.z;
    const float m[16]={cy,-sy*sp,sy*cp,0,0,cp,sp,0,-sy,-cy*sp,cy*cp,0,
        x*cy-z*sy,y*cp-(x*sy+z*cy)*sp,y*sp+(x*sy+z*cy)*cp,1};
    for(int i=0;i<16;i++)view_matrix[i]=m[i];
}
static void local_view(float *out,const float *local) {
    for(int col=0;col<4;col++)for(int row=0;row<4;row++)
        out[col*4+row]=view_matrix[row]*local[col*4]+view_matrix[4+row]*local[col*4+1]+view_matrix[8+row]*local[col*4+2]+view_matrix[12+row]*local[col*4+3];
}
static void lerp(CameraVertex *c,const CameraVertex *a,const CameraVertex *b,float t) {
    float *cc=&c->x;const float *aa=&a->x,*bb=&b->x;
    for(int i=0;i<10;i++) cc[i]=aa[i]+(bb[i]-aa[i])*t;
    c->mask=0;
}
static DrawVertex project(const CameraVertex *v) {
    float q=1/v->z;
    return (DrawVertex){256+v->x*332.553f*q,224-v->y*387.98f*q,v->z,v->u,v->v,
        (uint8_t)(int)clampf(v->r*draw_color[0]*128,0,255),(uint8_t)(int)clampf(v->g*draw_color[1]*128,0,255),
        (uint8_t)(int)clampf(v->b*draw_color[2]*128,0,255),(uint8_t)(int)(draw_alpha*v->alpha),q,(unsigned)(int)(clampf(v->f,0,1)*255)<<24};
}
static CameraVertex *project_base;static unsigned project_count;
static int lazy_lighting;static unsigned shade_offset;static float shade_tint[3],shade_material[3];
static DrawVertex *project_cache;static unsigned project_capacity;
static void start_projection(CameraVertex *base,unsigned count) {
    project_base=NULL;
    if(count>project_capacity){void *next=realloc(project_cache,count*sizeof(*project_cache));if(!next)return;project_cache=next;project_capacity=count;}
    project_base=base;project_count=count;platform_cache_reset();
    for(unsigned i=0;i<count;i++)base[i].mask&=~32u;
}
static inline void shade_cached(const CameraVertex *vertex) {
    if(!lazy_lighting||!project_base||(vertex->mask&64))return;
    unsigned index=(vertex->mask>>8)-1;
    CameraVertex *v=project_base+index;float light=0,distance2=v->r;
    if(rgame->flashlight&&distance2<400) {
        float distance=sqrtf(distance2);
        light=clampf((v->z/fmaxf(distance,.001f)-.9063078f)*18,0,1)*clampf(1-distance/20,0,1)*2.5f;
    }
    unsigned fi=(int)(distance2*.125f);float visible=fog_table[fi<4096?fi:4096],limit=rgame->time_of_day==2?1.7f:1.99f;
    const float *base=vertex_light[shade_offset+index];
    v->r=clampf(base[0]*shade_tint[0]+shade_material[0]*light,0,limit);
    v->g=clampf(base[1]*shade_tint[1]+shade_material[1]*light,0,limit);
    v->b=clampf(base[2]*shade_tint[2]+shade_material[2]*light,0,limit);v->f=visible;
    v->mask|=64;
}
static inline DrawVertex project_cached(const CameraVertex *v) {
    unsigned key=v->mask>>8;
    if(project_base&&key&&key<=project_count) {
        unsigned index=key-1;
        if(!(v->mask&32)){project_cache[index]=project(v);project_cache[index].cache_key|=index+1;project_base[index].mask|=32;}
        return project_cache[index];
    }
    return project(v);
}
typedef union {struct{float x,y,z,u,v,r,g,b,f,alpha;};float values[12];} ClipVertex __attribute__((aligned(16)));
static float clip_plane(const ClipVertex *v,int side) {
    switch(side){case 0:return v->z-.15f;case 1:return v->x+v->z*.7698f;case 2:return v->z*.7698f-v->x;
        case 3:return v->y+v->z*.57735f;default:return v->z*.57735f-v->y;}
}
static void clip_lerp(ClipVertex *out,const ClipVertex *a,const ClipVertex *b,float t) {
#ifdef _EE
    const float ratio[4] __attribute__((aligned(16)))={t,t,t,t};
    __asm__ volatile(
        "lqc2 $vf13,0(%3)\n lqc2 $vf14,0(%1)\n lqc2 $vf15,0(%2)\n"
        "vsub $vf15,$vf15,$vf14\n vmulx $vf15,$vf15,$vf13\n vadd $vf14,$vf14,$vf15\n sqc2 $vf14,0(%0)\n"
        "lqc2 $vf14,16(%1)\n lqc2 $vf15,16(%2)\n"
        "vsub $vf15,$vf15,$vf14\n vmulx $vf15,$vf15,$vf13\n vadd $vf14,$vf14,$vf15\n sqc2 $vf14,16(%0)\n"
        ::"r"(out),"r"(a),"r"(b),"r"(ratio):"memory");
#else
    for(int i=0;i<8;i++)out->values[i]=a->values[i]+(b->values[i]-a->values[i])*t;
#endif
    out->f=a->f+(b->f-a->f)*t;out->alpha=a->alpha+(b->alpha-a->alpha)*t;
}
static void clipped_triangle(int tex,const CameraVertex *a,const CameraVertex *b,const CameraVertex *c,unsigned outside) {
    ClipVertex buffers[2][12];ClipVertex *p=buffers[0],*temp=buffers[1];int n=3;
    memcpy(p,a,40);memcpy(p+1,b,40);memcpy(p+2,c,40);
    for(int side=0;side<5;side++) {
        if(!(outside&(1u<<side)))continue;
        int next=0;
        for(int i=0;i<n;i++) {
            const ClipVertex *va=p+i,*vb=p+(i+1)%n;float da=clip_plane(va,side),db=clip_plane(vb,side);
            if(da>=0) temp[next++]=*va;
            if((da>=0)!=(db>=0))clip_lerp(temp+next++,va,vb,da/(da-db));
        }
        n=next;if(n<3)return;ClipVertex *swap=p;p=temp;temp=swap;
    }
    DrawVertex projected[12];
    for(int i=0;i<n;i++){CameraVertex vertex;memcpy(&vertex,p+i,40);projected[i]=project(&vertex);}
    for(int i=1;i<n-1;i++) { DrawVertex v[3]={projected[0],projected[i],projected[i+1]};platform_triangle(tex,v);triangles++; }
}
static inline void triangle_vertices(int tex,const CameraVertex *a,const CameraVertex *b,const CameraVertex *c) {
    if(triangles>=TRIANGLE_BUDGET) return;
    unsigned ca=a->mask&31,cb=b->mask&31,cc=c->mask&31;if(ca&cb&cc)return;
    if(cull_faces&&a->x*(b->y*c->z-b->z*c->y)-a->y*(b->x*c->z-b->z*c->x)+a->z*(b->x*c->y-b->y*c->x)>=0)return;
    shade_cached(a);shade_cached(b);shade_cached(c);
    if(ca|cb|cc){clipped_triangle(tex,a,b,c,ca|cb|cc);return;}
    DrawVertex v[3]={project_cached(a),project_cached(b),project_cached(c)};platform_triangle(tex,v);triangles++;
}
#define triangle(texture,a,b,c) triangle_vertices((texture),&(a),&(b),&(c))
static CameraVertex floor_light[2048];static unsigned floor_light_count;
static float light_plane(const CameraVertex *v,int side) {
    switch(side){case 0:return v->z-.15f;case 1:return 20-v->z;case 2:return v->x+v->z*.4663077f;
        case 3:return v->z*.4663077f-v->x;case 4:return v->y+v->z*.4663077f;default:return v->z*.4663077f-v->y;}
}
static void collect_floor_light(const CameraVertex *a,const CameraVertex *b,const CameraVertex *c) {
    unsigned outside=0;
    for(int side=0;side<6;side++) {
        float da=light_plane(a,side),db=light_plane(b,side),dc=light_plane(c,side);
        if(da<0&&db<0&&dc<0)return;
        if(da<0||db<0||dc<0)outside|=1u<<side;
    }
    CameraVertex buffers[2][12];CameraVertex *p=buffers[0],*temp=buffers[1];int n=3;
    p[0]=*a;p[1]=*b;p[2]=*c;
    for(int side=0;side<6;side++)if(outside&(1u<<side)) {
        int next=0;
        for(int i=0;i<n;i++) {
            const CameraVertex *va=p+i,*vb=p+(i+1)%n;float da=light_plane(va,side),db=light_plane(vb,side);
            if(da>=0)temp[next++]=*va;
            if((da>=0)!=(db>=0))lerp(temp+next++,va,vb,da/(da-db));
        }
        n=next;if(n<3)return;CameraVertex *swap=p;p=temp;temp=swap;
    }
    for(int i=0;i<n;i++) {
        p[i].u=.5f+p[i].x/(p[i].z*.9326154f);p[i].v=.5f-p[i].y/(p[i].z*.9326154f);
        float fade=clampf((20-p[i].z)/5,0,1)*.65f;
        p[i].r=scene_flashlight_color[0]*fade;p[i].g=scene_flashlight_color[1]*fade;p[i].b=scene_flashlight_color[2]*fade;
        prepare_vertex(p+i);
    }
    for(int i=1;i<n-1&&floor_light_count+3<=2048;i++) {
        floor_light[floor_light_count++]=p[0];floor_light[floor_light_count++]=p[i];floor_light[floor_light_count++]=p[i+1];
    }
}
static void terrain_layers(CameraVertex v[4],int x,int z,int step) {
    float cumulative[4]={0};int first=1,width=rworld->header->width-1;
    for(int ground=0;ground<4;ground++) {
        float weights[4],maximum=0;int ox[4]={0,step,step,0},oz[4]={0,0,step,step};
        for(int i=0;i<4;i++) {
            int ix=x+ox[i],iz=z+oz[i];if(ix>=width)ix=width-1;if(iz>=width)iz=width-1;
            weights[i]=rworld->splat[(iz*width+ix)*4+ground];maximum=fmaxf(maximum,weights[i]);
        }
        if(maximum==0)continue;
        float repeat=win_scene?5:ground==0?3:ground==1?5:ground==2?4:7;
        float uv[4][2];terrain_uv_quad(uv,x,z,step,rworld->config[C_SCALE],rworld->config[C_SCALE+2],repeat);
        if(!first)platform_blend(4);
        for(int i=0;i<4;i++) {
            cumulative[i]+=weights[i];v[i].alpha=first?1:(cumulative[i]>0?weights[i]/cumulative[i]:0);
            v[i].u=uv[i][0];v[i].v=uv[i][1];
        }
        int texture=(int)rworld->config[C_GROUND+ground];triangle(texture,v[0],v[2],v[1]);triangle(texture,v[0],v[3],v[2]);
        if(!first)platform_blend(0);
        first=0;
    }
    for(int i=0;i<4;i++)v[i].alpha=1;
}
static CameraVertex terrain_interpolate(const CameraVertex v[4],int u,int z,int step) {
    CameraVertex out;const CameraVertex *a,*b,*c;float wa,wb,wc;
    if(u>=z){a=v;b=v+1;c=v+2;wa=1-u/(float)step;wb=(u-z)/(float)step;wc=z/(float)step;}
    else {a=v;b=v+2;c=v+3;wa=1-z/(float)step;wb=u/(float)step;wc=(z-u)/(float)step;}
    for(int k=0;k<10;k++)(&out.x)[k]=(&a->x)[k]*wa+(&b->x)[k]*wb+(&c->x)[k]*wc;
    prepare_vertex(&out);return out;
}
/* Fully fogged tiles need one opaque pass. Retain all fine edge samples so
 * their borders meet the original near terrain without cracks or skirts. */
static void distant_terrain(uint8_t distant[1024],int step) {
    memset(distant,0,1024);
    if(menu_scene||win_scene||density<=0||rworld->header->width!=513||rworld->header->height!=513)return;
    const int tile=16;float sx=rworld->config[C_SCALE],sz=rworld->config[C_SCALE+2],limit=2.4f/density;
    if(limit>=far_distance)return;
    float white[3]={1,1,1};int texture=(int)rworld->config[C_GROUND];
    int cx=(int)(rgame->feet.x/(sx*tile)),cz=(int)(rgame->feet.z/(sz*tile));
    int radius=(int)(far_distance/(sx*tile))+2;
    for(int z=cz-radius;z<=cz+radius;z++)for(int x=cx-radius;x<=cx+radius;x++) {
        if(x<0||z<0||x>=32||z>=32)continue;
        float dx=fmaxf(fmaxf(x*tile*sx-rgame->feet.x,rgame->feet.x-(x+1)*tile*sx),0);
        float dz=fmaxf(fmaxf(z*tile*sz-rgame->feet.z,rgame->feet.z-(z+1)*tile*sz),0);
        float distance=dx*dx+dz*dz;if(distance<limit*limit)continue;
        distant[z*32+x]=1;
        if(distance>far_distance*far_distance)continue;
        float forward=((x+.5f)*tile*sx-rgame->feet.x)*sy+((z+.5f)*tile*sz-rgame->feet.z)*cy;
        if(forward<-(sx+sz)*tile*.5f)continue;
        CameraVertex edge[32],centre;int n=0,gridx=x*tile,gridz=z*tile;
        for(int side=0;side<4;side++)for(int d=0;d<tile;d+=step) {
            int ix=gridx+(side==0?d:side==1?tile:side==2?tile-d:0);
            int iz=gridz+(side==0?0:side==1?d:side==2?tile:tile-d);
            Vec3 p={ix*sx,rworld->height[iz*rworld->header->width+ix]*rworld->config[C_SCALE+1],iz*sz};
            float rx=p.x-rgame->feet.x,ry=p.y-rgame->feet.y-rgame->eye,rz=p.z-rgame->feet.z,vz=rx*sy+rz*cy;
            edge[n]=(CameraVertex){.x=rx*cy-rz*sy,.y=ry*cp-vz*sp,.z=ry*sp+vz*cp,.r=1,.g=1,.b=1,.alpha=1};prepare_vertex(edge+n++);
        }
        int ix=gridx+tile/2,iz=gridz+tile/2;
        centre=camera((Vec3){ix*sx,rworld->height[iz*rworld->header->width+ix]*rworld->config[C_SCALE+1],iz*sz},0,0,0,1,0,white);centre.f=0;
        for(int i=0;i<n;i++)triangle(texture,centre,edge[(i+1)%n],edge[i]);
    }
}
static void terrain(void) {
    cull_faces=1;
    static CameraVertex cache[96*96];static unsigned stamps[96*96],serial;
    if(++serial==0){for(int i=0;i<96*96;i++)stamps[i]=0;serial=1;}
    int step=terrain_step(rgame,menu_scene,win_scene);float sx=rworld->config[C_SCALE],sz=rworld->config[C_SCALE+2];
    static uint8_t distant[1024];distant_terrain(distant,step);
    int radius=(int)(far_distance/(sx*step))+1,tx=(int)(rgame->feet.x/sx)/step,tz=(int)(rgame->feet.z/sz)/step;
    float white[3]={1,1,1};
    floor_light_count=0;int light=!menu_scene&&rgame->flashlight;
    static uint8_t paint_detail[256*256];static const World *paint_world;static int paint_step;
    if(paint_world!=rworld||paint_step!=step){memset(paint_detail,0,sizeof(paint_detail));paint_world=rworld;paint_step=step;}
    for(int z=(tz-radius)*step;z<(tz+radius)*step;z+=step) for(int x=(tx-radius)*step;x<(tx+radius)*step;x+=step) {
        if(x<0||z<0||x+step>=(int)rworld->header->width||z+step>=(int)rworld->header->height) continue;
        if(!menu_scene&&!win_scene&&rworld->header->width==513&&distant[(z/16)*32+x/16])continue;
        float dx=x*sx-rgame->feet.x,dz=z*sz-rgame->feet.z;
        if(dx*dx+dz*dz>far_distance*far_distance) continue;
        if(dx*sy+dz*cy<-(float)step*sx*2) continue;
        CameraVertex v[4];int ox[4]={0,step,step,0},oz[4]={0,0,step,step};
        float heights[4],minheight=1e9f,maxheight=-1e9f;
        for(int i=0;i<4;i++){heights[i]=rworld->height[(z+oz[i])*rworld->header->width+x+ox[i]]*rworld->config[C_SCALE+1];minheight=fminf(minheight,heights[i]);maxheight=fmaxf(maxheight,heights[i]);}
        if(!menu_scene) {
            float side=(fabsf(cy)+fabsf(sy))*step*sx;
            float maximum_z=((sp>=0?maxheight:minheight)-rgame->feet.y-rgame->eye)*sp+(dx*sy+dz*cy+side)*cp;
            if(fabsf(dx*cy-dz*sy)-side>maximum_z*.7698f)continue;
        }
        int layer=rworld->header->version==1?rworld->splat[z*(rworld->header->width-1)+x]:0;if(layer>3) layer=0;
        float repeat=win_scene?5:layer==0?3:layer==1?5:layer==2?4:7;
        float uv[4][2]={{0}};
        if(rworld->header->version==1)terrain_uv_quad(uv,x,z,step,sx,sz,repeat);
        for(int i=0;i<4;i++) {
            int ix=x+ox[i],iz=z+oz[i];Vec3 p={ix*sx,heights[i],iz*sz};
            float nx=0,ny=1,nz=0;
            if(win_scene) {
                nx=(world_height(rworld,p.x-sx,p.z)-world_height(rworld,p.x+sx,p.z))/(2*sx);
                nz=(world_height(rworld,p.x,p.z-sz)-world_height(rworld,p.x,p.z+sz))/(2*sz);
                float norm=sqrtf(nx*nx+1+nz*nz);nx/=norm;ny/=norm;nz/=norm;
            }
            int slot=((iz-(tz-radius)*step)/step)*96+(ix-(tx-radius)*step)/step;
            if(slot>=0&&slot<96*96) {
                if(stamps[slot]!=serial){cache[slot]=camera(p,0,0,nx,ny,nz,white);stamps[slot]=serial;}
                v[i]=cache[slot];v[i].u=uv[i][0];v[i].v=uv[i][1];
            }else v[i]=camera(p,uv[i][0],uv[i][1],nx,ny,nz,white);
        }
        if(rworld->header->version==1) {
            int texture=(int)rworld->config[C_GROUND+layer];triangle(texture,v[0],v[2],v[1]);triangle(texture,v[0],v[3],v[2]);
        }else {
            unsigned detail=16;
            if(!menu_scene&&!win_scene&&dx*dx+dz*dz<48*48&&step>=2&&step<=4) {
                int slot=(z/step)*256+x/step;
                if(!paint_detail[slot]) {
                    unsigned code=16;
                    if(terrain_paint_step(rworld,x,z,step)<step) {
                        code=32;
                        if(step==2)code|=1;
                        else for(int q=0;q<4;q++)if(terrain_paint_step(rworld,x+(q&1)*2,z+(q>>1)*2,2)==1)code|=1<<q;
                    }
                    paint_detail[slot]=code;
                }
                detail=paint_detail[slot];
            }
            if(detail==16)terrain_layers(v,x,z,step);
            else {
                CameraVertex nodes[25];int stride=detail&15?1:2;
                for(int zz=0;zz<=step;zz+=stride)for(int xx=0;xx<=step;xx+=stride)nodes[zz*5+xx]=terrain_interpolate(v,xx,zz,step);
                for(int zz=0;zz<step;zz+=2)for(int xx=0;xx<step;xx+=2) {
                    int unit=detail&(1<<((zz/2)*2+xx/2)),sub=unit?1:2;
                    for(int vz=zz;vz<zz+2;vz+=sub)for(int ux=xx;ux<xx+2;ux+=sub) {
                        CameraVertex fine[4]={nodes[vz*5+ux],nodes[vz*5+ux+sub],nodes[(vz+sub)*5+ux+sub],nodes[(vz+sub)*5+ux]};
                        terrain_layers(fine,x+ux,z+vz,sub);
                    }
                }
            }
        }
        if(light&&dx*dx+dz*dz<32*32){collect_floor_light(v,v+2,v+1);collect_floor_light(v,v+3,v+2);}
    }
}
static void skybox(void) {
    cull_faces=0;
    static const float corners[6][4][3]={
        {{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}},
        {{1,-1,-1},{-1,-1,-1},{-1,1,-1},{1,1,-1}},
        {{1,-1,1},{1,-1,-1},{1,1,-1},{1,1,1}},
        {{-1,-1,-1},{-1,-1,1},{-1,1,1},{-1,1,-1}},
        {{-1,1,1},{1,1,1},{1,1,-1},{-1,1,-1}},
        {{-1,-1,-1},{1,-1,-1},{1,-1,1},{-1,-1,1}}};
    float white[3]={1,1,1};
    for(int face=0;face<6;face++) {
        CameraVertex v[4];
        for(int j=0;j<4;j++) {
            Vec3 p={rgame->feet.x+corners[face][j][0]*10000,rgame->feet.y+rgame->eye+corners[face][j][1]*10000,rgame->feet.z+corners[face][j][2]*10000};
            v[j]=camera(p,j==1||j==2?1:0,j>=2?0:1,0,1,0,white);v[j].r=v[j].g=v[j].b=1;v[j].f=1;prepare_vertex(v+j);
        }
        int texture=win_scene?victory_sky[face]:scene_sky[rgame->time_of_day][face];
        triangle(texture,v[0],v[1],v[2]);triangle(texture,v[0],v[2],v[3]);
    }
}
typedef struct {unsigned index;float center[3],radius;} DrawMesh;
static DrawMesh *draw_meshes;static size_t draw_capacity;static unsigned draw_count;
static const World *draw_world;
static int prepare_mesh_bounds(void) {
    if(draw_world==rworld)return 1;
    if(rworld->header->meshes>draw_capacity) {
        void *next=realloc(draw_meshes,rworld->header->meshes*sizeof(*draw_meshes));
        if(!next)return 0;
        draw_meshes=next;draw_capacity=rworld->header->meshes;
    }
    draw_count=0;
    for(unsigned i=0;i<rworld->header->meshes;i++) {
        const Mesh *m=rworld->mesh+i;if(m->flags&(M_TREE|M_VIEW))continue;
        DrawMesh *d=draw_meshes+draw_count++;d->index=i;
        for(int k=0;k<3;k++)d->center[k]=(m->min[k]+m->max[k])*.5f;
        float x=m->max[0]-m->min[0],y=m->max[1]-m->min[1],z=m->max[2]-m->min[2];
        d->radius=sqrtf(x*x+z*z+((m->flags&(M_ENEMY|M_HOOVER))?y*y:0))*.5f;
    }
    draw_world=rworld;return 1;
}
static void meshes(void) {
    static CameraVertex *cache;static size_t capacity;
    if(!prepare_mesh_bounds())return;
    float enemy_pose[16] __attribute__((aligned(16))),hoover_pose[16] __attribute__((aligned(16)));
    float enemy_local[16] __attribute__((aligned(16))),hoover_local[16] __attribute__((aligned(16)));
    if(!menu_scene) {
        game_enemy_matrix(rworld,rgame,enemy_local);local_view(enemy_pose,enemy_local);
        game_hoover_matrix(rworld,rgame,hoover_local);local_view(hoover_pose,hoover_local);
    }
    uint32_t cached_offset=UINT32_MAX,cached_actor=UINT32_MAX;float white[3]={1,1,1},cached_color[3]={-1,-1,-1};
    for(unsigned entry=0;entry<draw_count;entry++) {
        const DrawMesh *bounds=draw_meshes+entry;unsigned i=bounds->index;
        const Mesh *m=rworld->mesh+i;
        if((m->flags&M_ENEMY)&&(rgame->state==ST_DEAD||rgame->state==ST_WON))continue;
        if((m->flags&M_DAY_TREE)&&rgame->time_of_day==2)continue;
        if((m->flags&M_COLLECT)&&(rgame->custards&(1<<m->id))) continue;
        float x=bounds->center[0],z=bounds->center[2],radius=bounds->radius;
        if(!menu_scene&&(m->flags&(M_ENEMY|M_HOOVER))) {
            const float *pose=m->flags&M_ENEMY?enemy_local:hoover_local;
            x=pose[0]*bounds->center[0]+pose[4]*bounds->center[1]+pose[8]*bounds->center[2]+pose[12];
            z=pose[2]*bounds->center[0]+pose[6]*bounds->center[1]+pose[10]*bounds->center[2]+pose[14];
        }
        float dx=x-rgame->feet.x,dz=z-rgame->feet.z;
        float mesh_far=menu_scene||density<=0?far_distance:fminf(far_distance,2.3f/density);
        if(dx*dx+dz*dz>(mesh_far+radius)*(mesh_far+radius)||dx*sy+dz*cy < -radius-2) continue;
        if(!menu_scene&&fabsf(dx*cy-dz*sy)>(dx*sy+dz*cy)*.77f+radius*1.27f) continue;
        if(!menu_scene&&!(m->flags&(M_ENEMY|M_HOOVER|M_DOOR))&&!tree_in_view(&(Tree){0,0,0,1,1,0},(float[6]){m->min[0],m->min[1],m->min[2],m->max[0],m->max[1],m->max[2]}))continue;
        if(m->vertices>capacity) {CameraVertex *next=realloc(cache,m->vertices*sizeof(*cache));if(!next) return;cache=next;capacity=m->vertices;}
        const Vertex *vertices=(const Vertex *)(rworld->blob+m->vertex_offset);
        const uint16_t *indices=(const uint16_t *)(rworld->blob+m->index_offset);
        const float *material=menu_scene?white:m->color;
        unsigned actor=(m->flags&(M_ENEMY|M_HOOVER|M_DOOR))|(m->id<<16);
        if(cached_offset!=m->vertex_offset||cached_actor!=actor||cached_color[0]!=material[0]||cached_color[1]!=material[1]||cached_color[2]!=material[2]) {
            platform_work(0,1);
            float local[16] __attribute__((aligned(16)))={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
            float matrix[16] __attribute__((aligned(16)));const float *pose=view_matrix;
            if(!menu_scene&&(m->flags&(M_ENEMY|M_HOOVER|M_DOOR))) {
                if(m->flags&M_ENEMY)pose=enemy_pose;
                else if(m->flags&M_HOOVER)pose=hoover_pose;
                else {
                    const float *d=rworld->config+101+m->id*12;
                    for(int col=0;col<4;col++)for(int row=0;row<3;row++)local[col*4+row]+=(d[row*4+col]-local[col*4+row])*rgame->door_open;
                    local_view(matrix,local);pose=matrix;
                }
            }
            if(!menu_scene)camera_vertices(cache,vertices,m->vertices,pose);
            else for(uint32_t j=0;j<m->vertices;j++) {
                const Vertex *v=vertices+j;Vec3 p={v->x,v->y,v->z};
                cache[j]=camera(p,v->u,v->v,v->nx,v->ny,v->nz,material);
                cache[j].mask|=(j+1)<<8;
            }
            cached_offset=m->vertex_offset;cached_actor=actor;
            for(int j=0;j<3;j++)cached_color[j]=material[j];
            platform_work(0,0);
        }
        for(int j=0;j<3;j++) draw_color[j]=menu_scene?m->color[j]:1;
        cull_faces=(m->flags&M_CULL)!=0;
        platform_cutout((m->flags&M_CUTOUT)!=0);
        lazy_lighting=!menu_scene;shade_offset=light_offsets?light_offsets[i]:0;
        for(int k=0;k<3;k++){shade_tint[k]=1;shade_material[k]=material[k];}
        start_projection(cache,m->vertices);platform_work(1,1);
        for(uint32_t j=0;j<m->triangles;j++,indices+=3) triangle(m->texture,cache[indices[0]],cache[indices[1]],cache[indices[2]]);
        platform_work(1,0);project_base=NULL;lazy_lighting=0;
    }
    platform_cutout(0);
    for(int j=0;j<3;j++) draw_color[j]=1;
}
typedef struct {unsigned index;float distance;} VisibleTree;
static int nearer_tree(const void *a,const void *b) {
    float delta=((const VisibleTree *)a)->distance-((const VisibleTree *)b)->distance;
    return delta<0?-1:delta>0;
}
static int tree_in_view(const Tree *t,const float bounds[6]) {
    float hx=(bounds[3]-bounds[0])*t->width*.5f,hy=(bounds[4]-bounds[1])*t->height*.5f,hz=(bounds[5]-bounds[2])*t->width*.5f;
    float dx=t->x+(bounds[3]+bounds[0])*t->width*.5f-rgame->feet.x;
    float dy=t->y+(bounds[4]+bounds[1])*t->height*.5f-rgame->feet.y-rgame->eye;
    float dz=t->z+(bounds[5]+bounds[2])*t->width*.5f-rgame->feet.z;
    float x=dx*cy-dz*sy,y=dy*cp-(dx*sy+dz*cy)*sp,z=dy*sp+(dx*sy+dz*cy)*cp;
    if(z-.15f+fabsf(sy*cp)*hx+fabsf(sp)*hy+fabsf(cy*cp)*hz<0)return 0;
    for(int sign=-1;sign<=1;sign+=2) {
        float radius=fabsf(sign*cy+.7698f*sy*cp)*hx+fabsf(.7698f*sp)*hy+fabsf(-sign*sy+.7698f*cy*cp)*hz;
        if(sign*x+.7698f*z+radius<0)return 0;
        radius=fabsf(-sign*sy*sp+.57735f*sy*cp)*hx+fabsf(sign*cp+.57735f*sp)*hy+fabsf(-sign*cy*sp+.57735f*cy*cp)*hz;
        if(sign*y+.57735f*z+radius<0)return 0;
    }
    return 1;
}
static void trees(void) {
    static VisibleTree *visible;static size_t visible_capacity;
    static CameraVertex *cache;static size_t capacity;
    if(rworld->header->trees>visible_capacity) {
        VisibleTree *next=realloc(visible,rworld->header->trees*sizeof(*next));if(!next)return;
        visible=next;visible_capacity=rworld->header->trees;
    }
    unsigned count=0,tree_work=0;
    // Reserve CPU work for the original enemy mesh during an encounter.
    // The closest trees keep priority; remaining trees retain leaf-card LODs.
    float enemy_dx=rgame->enemy.x-rgame->feet.x,enemy_dz=rgame->enemy.z-rgame->feet.z;
    float enemy_d2=enemy_dx*enemy_dx+enemy_dz*enemy_dz,enemy_dy=rgame->enemy.y+1-rgame->feet.y-rgame->eye;
    float forward=enemy_dx*sy+enemy_dz*cy,enemy_z=enemy_dy*sp+forward*cp,enemy_y=enemy_dy*cp-forward*sp;
    // Match rendering, rather than AI line-of-sight: an enemy behind a trunk
    // still submits its mesh to the GS and consumes the same CPU time.
    int enemy_on_screen=enemy_d2<900&&enemy_z>-2&&fabsf(enemy_dx*cy-enemy_dz*sy)<enemy_z*.77f+2.5f&&fabsf(enemy_y)<enemy_z*.58f+2.5f;
    unsigned tree_budget=win_scene?7000:enemy_on_screen?(enemy_d2<36?1200:2200):4000;
    // Beyond this distance the source exponential fog hides the geometry.
    float limit=fminf(far_distance,density>0?2.3f/density:far_distance);
    for(uint32_t i=0;i<rworld->header->trees;i++) {
        const Tree *t=rworld->tree+i;float dx=t->x-rgame->feet.x,dz=t->z-rgame->feet.z;
        float dist=dx*dx+dz*dz;
        if(dist>(limit+12)*(limit+12)||dx*sy+dz*cy<-12)continue;
        if(fabsf(dx*cy-dz*sy)>(dx*sy+dz*cy)*.77f+16)continue;
        if((unsigned)t->type>=3||!tree_in_view(t,tree_bounds[(unsigned)t->type]))continue;
        visible[count++]=(VisibleTree){i,dist};
    }
    qsort(visible,count,sizeof(*visible),nearer_tree);
    for(unsigned i=0;i<count;i++) {
        const Tree *t=rworld->tree+visible[i].index;uint32_t cached_offset=UINT32_MAX;float cached_color[3]={-1,-1,-1};
        float matrix[16] __attribute__((aligned(16))),local[16] __attribute__((aligned(16)))={t->width,0,0,0,0,t->height,0,0,0,0,t->width,0,t->x,t->y,t->z,1};
        local_view(matrix,local);
        // Keep the original close mesh; reduce leaf groups sooner on the PS2.
        float near=win_scene?20:8,medium=win_scene?60:20;
        unsigned kind=(unsigned)t->type;
        float center_y=t->y+(tree_bounds[kind][1]+tree_bounds[kind][4])*.5f*t->height-rgame->feet.y-rgame->eye;
        float distance=visible[i].distance+(win_scene?0:center_y*center_y);
        unsigned prototype=kind+(distance>medium*medium?6:distance>near*near?3:0);
        if(distance>(win_scene?110*110:30*30))prototype=kind+9;
        while(prototype<9&&tree_work+tree_cost[prototype]>tree_budget)prototype+=3;
        if(!win_scene&&prototype>=9&&(enemy_on_screen||tree_work+tree_cost[prototype]>tree_budget))prototype=kind+12;
        tree_work+=tree_cost[prototype];
        // Preserve complete trees when the CPU's frame budget is reached.
        if(triangles+tree_cost[prototype]*2>TRIANGLE_BUDGET-(win_scene?4800:0))continue;
        for(unsigned part=0;part<tree_count[prototype];part++) {
            unsigned j=tree_order[tree_first[prototype]+part];const Mesh *m=rworld->mesh+j;
            if(!tree_in_view(t,(float[6]){m->min[0],m->min[1],m->min[2],m->max[0],m->max[1],m->max[2]}))continue;
            if(m->vertices>capacity) {
                CameraVertex *next=realloc(cache,m->vertices*sizeof(*next));if(!next)return;cache=next;capacity=m->vertices;
            }
            unsigned tint=active_tree_colors[visible[i].index];float material[3];
            for(int k=0;k<3;k++)material[k]=m->color[k]*((tint>>(k*8))&255)/255.f;
            if(cached_offset!=m->vertex_offset||cached_color[0]!=material[0]||cached_color[1]!=material[1]||cached_color[2]!=material[2]) {
                platform_work(0,1);
                const Vertex *vertices=(const Vertex *)(rworld->blob+m->vertex_offset);
                camera_vertices(cache,vertices,m->vertices,matrix);
                cached_offset=m->vertex_offset;
                for(int k=0;k<3;k++)cached_color[k]=material[k];
                platform_work(0,0);
            }
            for(int k=0;k<3;k++)draw_color[k]=1;
            cull_faces=(m->flags&M_CULL)!=0;
            platform_cutout((m->flags&M_CUTOUT)!=0);
            const uint16_t *indices=(const uint16_t *)(rworld->blob+m->index_offset);
            lazy_lighting=1;shade_offset=light_offsets[j];
            for(int k=0;k<3;k++){shade_tint[k]=((tint>>(k*8))&255)/255.f;shade_material[k]=material[k];}
            start_projection(cache,m->vertices);platform_work(1,1);
            for(uint32_t k=0;k<m->triangles;k++,indices+=3)triangle(m->texture,cache[indices[0]],cache[indices[1]],cache[indices[2]]);
            platform_work(1,0);project_base=NULL;lazy_lighting=0;
        }
    }
    platform_cutout(0);
    for(int j=0;j<3;j++)draw_color[j]=1;
}
static void first_person(void) {
    static CameraVertex *cache;static unsigned capacity;
    float phase=fmodf(rgame->time/view_walk_duration,1)*VIEW_WALK_FRAMES;
    unsigned frame=(unsigned)phase;float fraction=phase-frame;
    // The original arm and flashlight are stored relative to the player camera.
    // Render with rworld depth so the hand remains occluded correctly by geometry.
    for(uint32_t i=0;i<rworld->header->meshes;i++) {
        const Mesh *m=rworld->mesh+i;if(!(m->flags&M_VIEW))continue;
        unsigned outside=31;
        for(int corner=0;corner<8;corner++) {
            float x=corner&1?m->max[0]:m->min[0],y=corner&2?m->max[1]:m->min[1],z=corner&4?m->max[2]:m->min[2];
            CameraVertex bound={.x=x,.y=y*cp-z*sp,.z=y*sp+z*cp};prepare_vertex(&bound);outside&=bound.mask;
        }
        if(outside)continue;
        if(m->vertices>capacity){CameraVertex *next=realloc(cache,m->vertices*sizeof(*next));if(!next)return;cache=next;capacity=m->vertices;}
        const Vertex *vertices=(const Vertex *)(rworld->blob+m->vertex_offset);
        const uint16_t *indices=(const uint16_t *)(rworld->blob+m->index_offset);
        for(int k=0;k<3;k++)draw_color[k]=m->color[k];
        cull_faces=(m->flags&M_CULL)!=0;
        for(unsigned j=0;j<m->vertices;j++) {
                Vertex posed=vertices[j];
                if(walk_frames&&m->id<sizeof(view_walk_offsets)/sizeof(view_walk_offsets[0])) {
                    unsigned offset=view_walk_offsets[m->id]+j;
                    const float *a=walk_frames+(frame*VIEW_WALK_VERTICES+offset)*6;
                    const float *b=walk_frames+(((frame+1)%VIEW_WALK_FRAMES)*VIEW_WALK_VERTICES+offset)*6;
                    posed.x=a[0]+(b[0]-a[0])*fraction;posed.y=a[1]+(b[1]-a[1])*fraction;posed.z=a[2]+(b[2]-a[2])*fraction;
                    posed.nx=a[3]+(b[3]-a[3])*fraction;posed.ny=a[4]+(b[4]-a[4])*fraction;posed.nz=a[5]+(b[5]-a[5])*fraction;
                }
                const Vertex *v=&posed;float diffuse=clampf(.65f+v->ny*.3f-v->nz*.2f,.3f,1);
                float brightness=view_walk_parts[m->id]?(rgame->time_of_day==0?1.35f:rgame->flashlight?1.0f:.25f):rgame->flashlight?.65f:.25f;
                cache[j]=(CameraVertex){v->x,v->y*cp-v->z*sp,v->y*sp+v->z*cp,v->u,v->v,
                                     brightness*diffuse,brightness*diffuse,brightness*diffuse,1,1,0};
                prepare_vertex(cache+j);
        }
        for(unsigned j=0;j<m->triangles;j++,indices+=3) {
            if(view_walk_parts[m->id]!=2){triangle(m->texture,cache[indices[0]],cache[indices[1]],cache[indices[2]]);continue;}
            // Hide the upper torso in first person while retaining the original hips and legs.
            CameraVertex polygon[4];int count=0;
            for(int edge=0;edge<3;edge++) {
                const CameraVertex *a=cache+indices[edge],*b=cache+indices[(edge+1)%3];
                float da=-.98f-(a->y*cp+a->z*sp),db=-.98f-(b->y*cp+b->z*sp);
                if(da>=0)polygon[count++]=*a;
                if((da>=0)!=(db>=0)){lerp(polygon+count,a,b,da/(da-db));prepare_vertex(polygon+count++);}
            }
            for(int edge=1;edge<count-1;edge++)triangle(m->texture,polygon[0],polygon[edge],polygon[edge+1]);
        }
    }
    for(int k=0;k<3;k++)draw_color[k]=1;
}
static void flashlight_cookie(void) {
    if(!floor_light_count)return;
    // Reuse the floor's exact triangles: no independent decal can cut through it.
    // The light shares the camera origin, so its cookie is linear in screen space.
    cull_faces=0;platform_blend(3);
    for(unsigned i=0;i<floor_light_count;i++)floor_light[i].f=1;
    for(unsigned i=0;i<floor_light_count;i+=3)triangle(scene_cookie,floor_light[i],floor_light[i+1],floor_light[i+2]);
    platform_blend(0);
}
static void grass(void) {
    if(rworld->header->version<2||!rgame->draw_grass)return;
    const uint16_t *patches=(const uint16_t *)(rworld->blob+(unsigned)rworld->config[127]);
    const uint8_t *samples=rworld->blob+(unsigned)rworld->config[128];
    int grid=rworld->header->version==3?(int)rworld->config[130]:1024;
    float scale=grid/((rworld->header->width-1)*rworld->config[C_SCALE]);
    float range=win_scene?rworld->config[131]:16;
    int rings=(int)(range*scale)+1,budget=win_scene?1200:320;
    int cx=(int)(rgame->feet.x*scale),cz=(int)(rgame->feet.z*scale),count=0;
    cull_faces=0;
    platform_cutout(1);int step=terrain_step(rgame,menu_scene,win_scene);
    // Instantiate only nearby source density cells; one shared grass texture.
    for(int ring=0;ring<rings;ring++)for(int dz=-ring;dz<=ring;dz++)for(int dx=-ring;dx<=ring;dx++) {
        if(abs(dx)!=ring&&abs(dz)!=ring)continue;
        int x=cx+dx,z=cz+dz;if(x<0||z<0||x>=grid||z>=grid)continue;
        unsigned patch=patches[(z/8)*(grid/8)+x/8];if(!patch)continue;
        unsigned density=samples[patch*64+(z%8)*8+x%8];if(!density)continue;
        unsigned seed=(unsigned)x*73856093u^(unsigned)z*19349663u;
        float px=(x+.15f+(seed&255)*.0027f)/scale,pz=(z+.15f+((seed>>8)&255)*.0027f)/scale;
        float rx=px-rgame->feet.x,rz=pz-rgame->feet.z,d=rx*rx+rz*rz;
        if(d>range*range||rx*sy+rz*cy<-1||fabsf(rx*cy-rz*sy)>(rx*sy+rz*cy)*.77f+2)continue;
        unsigned kind=(density&240)?1:0;const float *spec=active_grass[kind];
        float variation=(seed&255)/255.f,height=spec[2]+(spec[3]-spec[2])*variation;
        float half=(spec[0]+(spec[1]-spec[0])*variation)*.5f;
        float dry=.5f+.45f*sinf(px*.1f)*cosf(pz*.1f),color[3];
        for(int k=0;k<3;k++)color[k]=spec[4+k]*(1-dry)+spec[7+k]*dry;
        float bend=sinf(rgame->time*1.5f+(seed&31))*.06f;
        int billboard=active_grass_modes[kind]==0;
        float inverse_distance=billboard?1/sqrtf(fmaxf(d,.0001f)):0;
        for(int axis=0;axis<(billboard?1:2);axis++) {
            float ax=billboard?rz*inverse_distance*half:axis?0:half;
            float az=billboard?-rx*inverse_distance*half:axis?half:0;CameraVertex v[4];
            // Each bottom corner follows the visible floor, including slopes.
            float yl=terrain_surface_height(rworld,step,px-ax,pz-az)-.025f;
            float yr=terrain_surface_height(rworld,step,px+ax,pz+az)-.025f;
            Vec3 p[4]={{px-ax,yl,pz-az},{px+ax,yr,pz+az},{px+ax+bend,yr+height,pz+az},{px-ax+bend,yl+height,pz-az}};
            for(int i=0;i<4;i++){v[i]=camera(p[i],i==1||i==2?1:0,i<2?1:0,0,1,0,color);v[i].alpha=clampf((range*range-d)/(range*5),0,1);}
            int texture=(int)rworld->config[129];triangle(texture,v[0],v[1],v[2]);triangle(texture,v[0],v[2],v[3]);
        }
        if(++count>=budget){platform_cutout(0);return;}
    }
    platform_cutout(0);
}
static void image(int tex,float x,float y,float width,float height,uint32_t tint) {platform_sprite(tex,x,y,width,height,0,0,1,1,tint);}
static void text(float x,float y,const char *str,uint32_t color) {
    int tex=(int)rworld->config[C_FONT];float start=x;
    for(;*str;str++) {unsigned char c=*str;if(c=='\n') {y+=18;x=start;continue;}if(c>=32&&c<128) platform_sprite(tex,x,y,8,16,(c%16)/16.f,(c/16-2)/8.f,1/16.f,1/8.f,color);x+=8;}
}
static void small_text(float x,float y,const char *str,uint32_t color) {
    int tex=(int)rworld->config[C_FONT];
    for(;*str;str++,x+=4){unsigned c=(unsigned char)*str;if(c>=32&&c<128)platform_sprite(tex,x,y,4,8,(c%16)/16.f,(c/16-2)/8.f,1/16.f,1/8.f,color);}
}
static unsigned particle_random(unsigned value) {value^=value>>16;value*=0x7feb352du;value^=value>>15;value*=0x846ca68bu;return value^(value>>16);}
static float particle_unit(unsigned value) {return (particle_random(value)&65535)/65535.f;}
static void menu_particles(void) {
    cull_faces=0;
    for(unsigned i=0;i<sizeof(menu_emitters)/sizeof(menu_emitters[0]);i++) {
        const MenuEmitter *emitter=menu_emitters+i;platform_blend(emitter->additive?1:2);
        for(unsigned j=0;j<24;j++) {
            // Gameplay time stops on menu screens; their own clock keeps emitting.
            unsigned seed=i*101+j*17+2013;float age=fmodf(rgame->state_time+particle_unit(seed)*emitter->life,emitter->life);
            float phase=age/emitter->life*4;int key=(int)phase;if(key>3)key=3;float blend=phase-key;
            float color[4];for(int k=0;k<4;k++)color[k]=emitter->color[key][k]*(1-blend)+emitter->color[key+1][k]*blend;
            if(color[3]<.008f)continue;
            float size=fmaxf(.05f,emitter->size[0]+particle_unit(seed+1)*(emitter->size[1]-emitter->size[0])+emitter->grow*age)*.5f;
            Vec3 center={emitter->position[0]+(particle_unit(seed+2)*2-1)*emitter->spread[0]+emitter->velocity[0]*age+emitter->force[0]*age*age*.5f,
                         emitter->position[1]+emitter->velocity[1]*age+emitter->force[1]*age*age*.5f,
                         emitter->position[2]+(particle_unit(seed+3)*2-1)*emitter->spread[2]+emitter->velocity[2]*age+emitter->force[2]*age*age*.5f};
            CameraVertex vertices[4];float white[3]={1,1,1};
            for(int k=0;k<4;k++) {
                float x=(k==1||k==2?1:-1)*size,y=(k>=2?1:-1)*size;
                Vec3 p={center.x+menu_matrix[0]*x+menu_matrix[1]*y,center.y+menu_matrix[4]*x+menu_matrix[5]*y,center.z+menu_matrix[8]*x+menu_matrix[9]*y};
                vertices[k]=camera(p,k==1||k==2?1:0,k>=2?0:1,0,1,0,white);
                vertices[k].r=color[0]*2;vertices[k].g=color[1]*2;vertices[k].b=color[2]*2;
                vertices[k].f=1; // Preserve the particle shader's emission/opacity ramp.
            }
            draw_alpha=(unsigned)clampf(color[3]*128,0,128);
            triangle(emitter->texture,vertices[0],vertices[1],vertices[2]);triangle(emitter->texture,vertices[0],vertices[2],vertices[3]);
        }
    }
    draw_alpha=128;platform_blend(0);
}
static void original_menu(const Game *g,int connected) {
    const uint32_t white=0x80808080,gray=0x80606060;
    rworld=menu_world;int bank=g->state==ST_HELP?1:g->state==ST_CREDITS?2:0;
    platform_world(rworld,bank);platform_clear(0,0,0);
    if(bank) {
        image((int)rworld->config[bank==1?86:87],0,0,512,448,white);
        // Only the platform-specific return hint differs from the source image.
        image(0,328,0,184,22,0x80000000);text(336,3,"X / O: BACK",white);
        platform_finish();return;
    }
    menu_scene=1;menu_matrix=menu_camera[g->menu_page];
    Game view=*g;view.feet=(Vec3){menu_matrix[3],menu_matrix[7],menu_matrix[11]};
    view.eye=0;view.flashlight=0;view.time_of_day=2;
    view.yaw=atan2f(menu_matrix[2],menu_matrix[10]);view.pitch=asinf(menu_matrix[6]);rgame=&view;
    sy=sinf(view.yaw);cy=cosf(view.yaw);sp=sinf(view.pitch);cp=cosf(view.pitch);
    far_distance=140;density=rworld->config[C_ENV];
    for(int i=0;i<3;i++){fog[i]=rworld->config[C_ENV+1+i];ambient[i]=rworld->config[C_ENV+4+i];}
    if(density!=cached_density){for(int i=0;i<4097;i++)fog_table[i]=expf(-density*density*i*8);cached_density=density;}
    platform_fog((uint8_t)(fog[0]*255),(uint8_t)(fog[1]*255),(uint8_t)(fog[2]*255));
    if(platform_static_begin(g->menu_page)) {
        terrain();platform_profile(1);menu_baked=1;meshes();menu_baked=0;platform_static_end();
    }else platform_profile(1);
    platform_profile(2);menu_particles();platform_profile(3);
    image((int)rworld->config[C_TITLE],rworld->config[95]*512,rworld->config[96]*448,rworld->config[97]*512,rworld->config[98]*448,white);
    if(g->menu_page==MENU_SETTINGS) {
        const char *levels[]={"Fastest","Fast","Simple","Good","Beautiful","Fantastic"};
        image(0,5,5,100,443,0x48000000);small_text(27,8,"Graphics Resolution",white);
        for(int i=0;i<6;i++) {
            image(0,10,20+i*15,40,10,i==g->menu?0x80606060:0x80303030);
            image(0,11,20+i*15,38,1,0x80505050);
            small_text(12,21+i*15,levels[i],white);
            if(i==g->quality)small_text(55,21+i*15,"*",white);
        }
    }
    int selected=g->menu_page==MENU_SETTINGS?0:g->menu;
    if(g->menu_page!=MENU_SETTINGS||g->menu==6) {
        unsigned id=menu_button_ids[g->menu_page][selected];float minx=512,miny=448,maxy=0;
        for(uint32_t i=0;i<rworld->header->meshes;i++) {
            const Mesh *m=rworld->mesh+i;if(m->id!=id)continue;
            const Vertex *v=(const Vertex *)(rworld->blob+m->vertex_offset);float white_color[3]={1,1,1};
            for(uint32_t j=0;j<m->vertices;j++) {
                CameraVertex c=camera((Vec3){v[j].x,v[j].y,v[j].z},0,0,0,1,0,white_color);
                if(c.z<.15f)continue;
                DrawVertex p=project(&c);
                minx=fminf(minx,p.x);miny=fminf(miny,p.y);maxy=fmaxf(maxy,p.y);
            }
        }
        text(fmaxf(2,minx-14),(miny+maxy)*.5f-8,">",white);
    }
    if(g->state==ST_NETWORK) {
        image(0,32,150,448,136,0x80101010);
        text(48,170,g->menu_page==MENU_COOP?"CO-OP":"VERSUS",white);
        text(48,205,g->menu_page==MENU_COOP?"Could not start co-op.\nCheck NET.CFG, the network\nadapter, cable and server.\n\nO: BACK":"Network play is not available\nin this build.\n\nO: BACK",white);
    }
    if(!connected)text(40,425,"CONNECT CONTROLLER TO PORT 1",gray);
    menu_scene=0;platform_profile(4);platform_finish();platform_profile(5);
}
// Co-op teammates are drawn as flat, coloured billboards (the original player
// model is not part of the converted rworld). Hidden when a wall is in between.
static void peers(void) {
    static const uint32_t colors[4]={0x780000FF,0x7800C800,0x78FF8000,0x78C000C0};
    static const char *names[4]={"P1","P2","P3","P4"};
    float eye[3]={rgame->feet.x,rgame->feet.y+rgame->eye,rgame->feet.z};(void)eye;
    for(int i=0;i<4;i++) {
        const Peer *p=rgame->peer+i;if(!p->active)continue;
        float top[3]={p->feet.x,p->feet.y+((p->flags&2)?1.0f:1.75f),p->feet.z},bottom[3]={p->feet.x,p->feet.y,p->feet.z};
        float vt[3],vb[3];
        for(int k=0;k<3;k++) {
            const float *pt=k==0?top:bottom;float *out=k==0?vt:vb;
            out[0]=view_matrix[0]*pt[0]+view_matrix[4]*pt[1]+view_matrix[8]*pt[2]+view_matrix[12];
            out[1]=view_matrix[1]*pt[0]+view_matrix[5]*pt[1]+view_matrix[9]*pt[2]+view_matrix[13];
            out[2]=view_matrix[2]*pt[0]+view_matrix[6]*pt[1]+view_matrix[10]*pt[2]+view_matrix[14];
            if(k==1)break;
        }
        if(vt[2]<.4f||vb[2]<.4f)continue;
        Vec3 from={rgame->feet.x,rgame->feet.y+rgame->eye,rgame->feet.z},to={p->feet.x,p->feet.y+1.2f,p->feet.z};
        if(world_occluded(rworld,rgame,from,to))continue;
        float sx=256+vt[0]*332.553f/vt[2],ty=224-vt[1]*387.98f/vt[2],by=224-vb[1]*387.98f/vb[2];
        float w=(by-ty)*.38f;if(w<3)w=3;
        if(sx<-w||sx>512+w)continue;
        image(0,sx-w*.5f,ty,w,by-ty,colors[i]);
        small_text(sx-4,ty-10,names[i],0x80FFFFFF);
    }
}
void render_frame(const World *w,const Game *g,int connected) {
    triangles=0;
    cull_faces=0;
    win_scene=0;
    platform_profile(0);
    if(g->state==ST_MENU||g->state==ST_HELP||g->state==ST_CREDITS||g->state==ST_NETWORK) {original_menu(g,connected);return;}
    Game fixed;
    win_scene=g->state==ST_WINSCENE;
    if(win_scene) {
        if(!victory_world)return;
        fixed=*g;fixed.feet=(Vec3){victory_camera[0],victory_camera[1],victory_camera[2]};
        fixed.yaw=victory_camera[3];fixed.pitch=victory_camera[4];fixed.eye=0;
        fixed.time_of_day=0;fixed.flashlight=fixed.map=0;
        fixed.draw_grass=1;fixed.quality=3;fixed.time=0;fixed.custards=0;
        g=&fixed;w=victory_world;
    }
    active_sun=win_scene?victory_sun:scene_sun[g->time_of_day];
    active_tree_colors=win_scene?victory_tree_colors:scene_tree_colors;
    active_grass=win_scene?victory_grass:source_grass;
    active_grass_modes=win_scene?victory_grass_modes:source_grass_modes;
    platform_world(w,g->time_of_day+1);
    rworld=w;rgame=g;triangles=0;sy=sinf(g->yaw);cy=cosf(g->yaw);sp=sinf(g->pitch);cp=cosf(g->pitch);
    const float *env=w->config+C_ENV+g->time_of_day*7;density=env[0];
    for(int i=0;i<3;i++) {fog[i]=env[1+i];ambient[i]=env[4+i];}
    if(density!=cached_density) {for(int i=0;i<4097;i++) fog_table[i]=expf(-density*density*i*8);cached_density=density;}
    if(!cache_lights())return;
    prepare_view_matrix();
    far_distance=(g->time_of_day==0?160:g->time_of_day==1?110:75)*(.7f+g->quality*.1f);
    platform_clear((uint8_t)(fog[0]*255),(uint8_t)(fog[1]*255),(uint8_t)(fog[2]*255));
    platform_fog((uint8_t)(fog[0]*255),(uint8_t)(fog[1]*255),(uint8_t)(fog[2]*255));
    const uint32_t white=0x80808080,gray=0x80606060;
    if(win_scene) {
        // Replay the fixed camera's GS geometry; this remains a depth-tested 3D scene.
        if(platform_static_begin(100)) {
            skybox();terrain();int ground=triangles;
            meshes();int models=triangles-ground;
            trees();int foliage=triangles-ground-models;
            grass();
            printf("STPS VICTORY GEOMETRY ground=%d models=%d trees=%d grass=%d total=%d\n",ground,models,foliage,triangles-ground-models-foliage,triangles);
            platform_static_end();
        }
        image((int)w->config[C_WON],0,0,512,448,white);
        image(0,0,0,400,12,0x60484848);
        small_text(128,2,"Return to the main menu",white);
        if(!connected)small_text(170,432,"CONNECT CONTROLLER TO PORT 1",white);
    } else {
        skybox();first_person();terrain();platform_profile(1);meshes();platform_profile(2);trees();grass();if(g->coop)peers();flashlight_cookie();platform_profile(3);
        if(g->state==ST_WON)image((int)w->config[C_WON],victory_notice[0]*512,victory_notice[1]*448,victory_notice[2]*512,victory_notice[3]*448,white);
        if(g->state!=ST_DEAD&&g->state!=ST_WON&&g->state_time<5)for(int i=0;i<2;i++)image((int)scene_intro[i][0],scene_intro[i][1]*512,scene_intro[i][2]*448,scene_intro[i][3]*512,scene_intro[i][4]*448,white);
        if(g->pickup_time>0&&g->collected>0&&g->collected<10){const float *p=source_pickups[g->collected-1];image((int)p[0],p[1]*512,p[2]*448,p[3]*512,p[4]*448,white);}
        if(g->state!=ST_WON&&g->scare_time>0&&g->scare_image>0) {int config=g->scare_image<=4?C_POPUP+g->scare_image-1:125+g->scare_image-5;image((int)w->config[config],0,0,512,448,white);}
        if(g->enemy_attacking&&g->state!=ST_DEAD&&g->state!=ST_WON)image(source_damage_border,0,0,512,448,white);
        if(g->state==ST_DEAD)image((int)w->config[C_DEAD],0,0,512,448,white);
        if(g->map) {image((int)w->config[C_MAP],32,16,448,400,white);text(48,420,"TRIANGULO: CERRAR MAPA",white);}
        if(g->state==ST_PAUSE) {image(0,80,160,352,128,0x80080808);text(168,180,"PAUSA",white);text(104,218,"START: CONTINUAR\nSELECT: MENU PRINCIPAL",gray);}
        if(!connected) text(40,390,"MANDO DESCONECTADO",white);
    }
    platform_profile(4);platform_finish();platform_profile(5);
}

/* ===== ps2_main.c ===== */
extern unsigned char world_start[],world_end[];
extern unsigned char menu_world_start[],menu_world_end[];
extern unsigned char victory_world_start[],victory_world_end[];
extern unsigned char player_walk_start[],player_walk_end[];
extern unsigned char sio2_start[],sio2_end[],pad_start[],pad_end[],audsrv_start[],audsrv_end[];
static GSGLOBAL *gs;static GSTEXTURE textures[128];static World world,menu_world,victory_world;static Game game;
static unsigned char texture_bits[128][2];
typedef struct {u64 rgbaq,st,xyz;} PackedPoint;
static PackedPoint batch[1024] __attribute__((aligned(64)));
static int batch_count,batch_texture=-1,notice_texture=-1;
static PackedPoint packed_cache[2048] __attribute__((aligned(64)));
static unsigned packed_stamp[2048],packed_serial;
static GSQUEUE menu_cache;static int menu_cache_ready,menu_cache_key=-1,blend_mode;
static char pad_buffer[256] __attribute__((aligned(64)));
static uint32_t previous;static int analog_requested;
static int audio_ready;static int sound_pos[9];
static int enemy_test_enabled,enemy_voice_starts,enemy_voice_ignored,enemy_voice_finished,enemy_side_shifts;
static u64 nearby_audible_until;static int nearby_pending;
static int profile_enabled;
static int menu_music_pos,was_menu_audio=-1;
static int16_t audio_chunk[512] __attribute__((aligned(64)));
static void audio_pump(void);
static void upload_texture(GSTEXTURE *texture) {
    // The standalone gsKit uploader allocates a temporary packet as large as
    // the residual image block. Use references in our existing GS queue so
    // scene changes do not need another 64+ KiB of contiguous EE heap.
    gsKit_setup_tbw(texture);
    gsKit_texture_send_inline(gs,texture->Mem,texture->Width,texture->Height,
                             texture->Vram,texture->PSM,texture->TBW,GS_CLUT_NONE);
}
static u64 work_begin[2],work_total[2];
void platform_work(int kind,int begin) {
    if(!profile_enabled)return;
    u64 now=GetTimerSystemTime();if(begin)work_begin[kind]=now;else work_total[kind]+=now-work_begin[kind];
}
void platform_cache_reset(void) {
    if(++packed_serial==0){memset(packed_stamp,0,sizeof(packed_stamp));packed_serial=1;}
}
void platform_profile(int phase) {
    static u64 last,total[6];static unsigned frames;
    if(!profile_enabled)return;
    u64 now=GetTimerSystemTime();
    if(phase==0){last=now;return;}
    total[phase]+=now-last;last=now;
    if(phase==5&&++frames==60) {
        float scale=1000.f/(kBUSCLK*60.f);
        printf("STPS PROFILE state=%d terrain=%.2f meshes=%.2f trees=%.2f ui=%.2f gs=%.2f ms\n",game.state,
            total[1]*scale,total[2]*scale,total[3]*scale,total[4]*scale,total[5]*scale);
        printf("STPS WORK vertices=%.2f triangles=%.2f ms\n",work_total[0]*scale,work_total[1]*scale);
        memset(work_total,0,sizeof(work_total));
        memset(total,0,sizeof(total));frames=0;
    }
}

static void flush(void) {
    // A menu frame may outlast audsrv's small IOP ring buffer. Feed it during
    // rendering as well, without waiting for the next gameplay update.
    audio_pump();
    if(batch_count) {
        GSTEXTURE *t=textures+batch_texture;int tw=texture_bits[batch_texture][0],th=texture_bits[batch_texture][1];
        gsKit_set_texfilter(gs,t->Filter);
        // REGLIST carries the same ST/Q, color and depth in half the A+D bytes.
        unsigned payload=(batch_count*3+1)/2,qsize=3+payload;
        u64 *p=gsKit_heap_alloc(gs,qsize,qsize*16,GIF_AD);
        *p++=GIF_TAG_AD(2);*p++=GIF_AD;
        // Texture alpha is needed by alpha testing even when blending is off.
        *p++=GS_SETREG_TEX0(t->Vram/256,t->TBW,t->PSM,tw,th,1,0,0,0,0,0,GS_CLUT_STOREMODE_NOLOAD);
        *p++=GS_TEX0_1+gs->PrimContext;
        *p++=GS_SETREG_PRIM(GS_PRIM_PRIM_TRIANGLE,1,1,gs->PrimFogEnable,gs->PrimAlphaEnable,gs->PrimAAEnable,0,gs->PrimContext,0);*p++=GS_PRIM;
        *p++=GIF_TAG(batch_count,1,0,0,GSKIT_GIF_FLG_REGLIST,3);
        *p++=GS_RGBAQ|(GS_ST<<4)|(GS_XYZF2<<8);
        memcpy(p,batch,batch_count*sizeof(*batch));
        if(batch_count&1)p[batch_count*3]=0;
    }
    batch_count=0;
}
void platform_blend(int mode) {
    flush();blend_mode=mode;gsKit_set_primalpha(gs,mode==1||mode==3?GS_SETREG_ALPHA(0,2,0,1,0):GS_SETREG_ALPHA(0,1,0,1,0),0);
    u64 *packet=gsKit_heap_alloc(gs,1,16,GIF_AD);
    packet[0]=GIF_TAG_AD(1);packet[1]=GIF_AD;
    packet[2]=GS_SETREG_ZBUF(gs->ZBuffer/8192,gs->PSMZ&15,mode!=0);packet[3]=GS_ZBUF_1+gs->PrimContext;
}
void platform_cutout(int enabled) {
    static int active;
    if(active==enabled)return;
    flush();active=enabled;
    // Unity foliage uses an opaque cutout. Low-alpha pixels must not write
    // depth or blend the white fog over branches that are behind them.
    gs->PrimAlphaEnable=enabled?GS_SETTING_OFF:GS_SETTING_ON;
    gs->Test->AREF=enabled?64:1;
    gsKit_set_test(gs,GS_ATEST_ON);
}
void platform_fog(uint8_t r,uint8_t g,uint8_t b) {
    flush();gs->PrimFogEnable=1;
    u64 *p=gsKit_heap_alloc(gs,1,16,GIF_AD);p[0]=GIF_TAG_AD(1);p[1]=GIF_AD;
    p[2]=(u64)r|((u64)g<<8)|((u64)b<<16);p[3]=GS_FOGCOL;
}
int platform_static_begin(int key) {
    flush();gsKit_queue_exec_real(gs,gs->Os_Queue);
    if(!menu_cache_ready){gsKit_queue_init(gs,&menu_cache,GS_PERSISTENT,1536*1024);menu_cache_ready=1;}
    gsKit_queue_set(gs,&menu_cache);
    if(key==menu_cache_key){gsKit_queue_exec_real(gs,&menu_cache);gsKit_queue_set(gs,gs->Os_Queue);return 0;}
    gsKit_queue_reset(&menu_cache);menu_cache_key=key;return 1;
}
void platform_static_end(void) {
    flush();
    printf("STPS CACHE key=%d bytes=%lu capacity=%lu\n",menu_cache_key,
        (unsigned long)((u8 *)menu_cache.pool_cur-(u8 *)menu_cache.pool[menu_cache.dbuf]),
        (unsigned long)((u8 *)menu_cache.pool_max[menu_cache.dbuf]-(u8 *)menu_cache.pool[menu_cache.dbuf]));
    gsKit_queue_exec_real(gs,&menu_cache);gsKit_queue_set(gs,gs->Os_Queue);
}
void platform_world(const World *w,int bank) {
    static const World *resident;static int resident_bank=-1;
    if(w==resident&&bank==resident_bank)return;
    menu_cache_key=-1;
    notice_texture=-1;
    dmaKit_wait_fast();gsKit_vram_clear(gs);
    for(uint32_t i=0;i<w->header->textures;i++) {
        if(w==&world) {
            // This title belongs to the old flat menu. The actual menu has
            // its own asset bank; release its 256 KiB for source foliage.
            if(i==(unsigned)w->config[C_TITLE])continue;
            int pickup=0;for(int p=1;p<9;p++)if(i==(unsigned)source_pickups[p][0])pickup=1;
            if(pickup)continue;
            int sky=0,current=0;
            for(int day=0;day<3;day++)for(int face=0;face<6;face++)if(i==scene_sky[day][face]){sky=1;if(day==bank-1)current=1;}
            if(sky&&!current)continue;
        }
        if(w==&menu_world) {
            int help=(int)w->config[86],credits=(int)w->config[87],font=(int)w->config[C_FONT];
            if(bank==0&&(i==(unsigned)help||i==(unsigned)credits))continue;
            if(bank&&i!=0&&i!=(unsigned)font&&i!=(unsigned)(bank==1?help:credits))continue;
        }
        const Texture *d=w->texture+i;GSTEXTURE *t=textures+i;memset(t,0,sizeof(*t));t->Width=d->width;t->Height=d->height;
        for(int axis=0;axis<2;axis++){unsigned n=axis?d->height:d->width,bits=0;while((1u<<bits)<n)bits++;texture_bits[i][axis]=bits;}
        t->PSM=GS_PSM_CT32;t->Filter=GS_FILTER_LINEAR;t->Mem=(u32 *)(w->blob+d->offset);
        t->Vram=gsKit_vram_alloc(gs,gsKit_texture_size(t->Width,t->Height,t->PSM),GSKIT_ALLOC_USERBUFFER);
        if(t->Vram==GSKIT_ALLOC_ERROR){printf("STPS VRAM ERROR texture=%lu bank=%d\n",(unsigned long)i,bank);SleepThread();}
        upload_texture(t);
    }
    if(w==&world)for(int p=1;p<9;p++) {
        unsigned id=(unsigned)source_pickups[p][0];textures[id]=textures[(unsigned)source_pickups[0][0]];
        textures[id].Mem=(u32 *)(w->blob+w->texture[id].offset);
        texture_bits[id][0]=texture_bits[(unsigned)source_pickups[0][0]][0];texture_bits[id][1]=texture_bits[(unsigned)source_pickups[0][0]][1];
    }
    gsKit_queue_exec(gs);dmaKit_wait_fast();
    resident=w;resident_bank=bank;
    printf("STPS TEXTURES scene=%s bank=%d vram=%lu\n",w==&menu_world?"menu":w==&victory_world?"victory":"game",bank,(unsigned long)gs->CurrentPointer);
}
void platform_clear(uint8_t r,uint8_t g,uint8_t b) {
    flush();
    // Clearing must overwrite both buffers regardless of transparent UI state.
    gs->PrimAlphaEnable=GS_SETTING_OFF;
    gs->PrimFogEnable=0;
    gsKit_clear(gs,GS_SETREG_RGBAQ(r,g,b,128,0));
    gs->PrimAlphaEnable=GS_SETTING_ON;
    gsKit_set_primalpha(gs,GS_SETREG_ALPHA(0,1,0,1,0),0);
    gsKit_set_test(gs,GS_ZTEST_ON);
}
void platform_triangle(int tex,const DrawVertex *v) {
    if(batch_texture!=tex||batch_count+3>1023) {flush();batch_texture=tex;}
    for(int i=0;i<3;i++) {
        PackedPoint *p=batch+batch_count++;unsigned key=v[i].cache_key&65535;
        if(key&&key<=2048&&packed_stamp[key-1]==packed_serial){*p=packed_cache[key-1];continue;}
        float q=v[i].q,tq=blend_mode==3?1:q;
        union {u64 raw;struct{uint32_t rgba;float q;} value;} color;
        union {u64 raw;struct{float s,t;} value;} uv;
        memcpy(&color.value.rgba,&v[i].r,4);color.value.q=tq;
        uv.value.s=v[i].u*tq;uv.value.t=v[i].v*tq;p->rgbaq=color.raw;p->st=uv.raw;
        unsigned x=gsKit_float_to_int_x(gs,v[i].x),y=gsKit_float_to_int_y(gs,v[i].y*gs->Height/448.f);
        uint32_t depth=(int)(.15f*q*65534);
        // Clipped light triangles need a small GS depth bias after XYZ quantization.
        if(blend_mode==3||blend_mode==4)depth=depth>65531?65535:depth+4;
        p->xyz=(u64)x|((u64)y<<16)|((u64)depth<<32)|((u64)(v[i].cache_key>>24)<<56);
        if(key&&key<=2048){packed_cache[key-1]=*p;packed_stamp[key-1]=packed_serial;}
    }
}
void platform_sprite(int tex,float x,float y,float width,float height,float u,float v,float uw,float vh,uint32_t color) {
    flush();float ys=gs->Height/448.f;GSTEXTURE *t=textures+tex;gs->PrimFogEnable=0;
    // All nine notices reuse one VRAM slot. Upload only the active notice.
    if(game.state==ST_PLAY||game.state==ST_PAUSE||game.state==ST_DEAD)for(int p=0;p<9;p++) {
        if(tex==(int)source_pickups[p][0]&&notice_texture!=tex){upload_texture(t);notice_texture=tex;break;}
    }
    gsKit_prim_sprite_texture(gs,t,x,y*ys,u*t->Width,v*t->Height,(x+width),(y+height)*ys,(u+uw)*t->Width,(v+vh)*t->Height,65535,
        GS_SETREG_RGBAQ(color&255,(color>>8)&255,(color>>16)&255,(color>>24)&255,0));gs->PrimFogEnable=1;
}
void platform_finish(void) {flush();gsKit_queue_exec(gs);gsKit_sync_flip(gs);}
static int module(void *start,void *end,const char *name) {
    int result=0,id=SifExecModuleBuffer(start,(unsigned char *)end-(unsigned char *)start,0,NULL,&result);
    printf("STPS MODULE %s id=%d result=%d\n",name,id,result);return id>=0&&result>=0;
}
static float stick(unsigned char byte) {
    float x=((int)byte-128)/127.f;if(fabsf(x)<.18f) return 0;
    return (x>0?1:-1)*(fabsf(x)-.18f)/.82f;
}
static Input read_input(void) {
    Input in={0};int state=padGetState(0,0);
    if(state==PAD_STATE_DISCONN) {previous=0;analog_requested=0;return in;}
    if(state!=PAD_STATE_STABLE&&state!=PAD_STATE_FINDCTP1) return in;
    if(!analog_requested) {padSetMainMode(0,0,PAD_MMODE_DUALSHOCK,PAD_MMODE_LOCK);analog_requested=1;return in;}
    struct padButtonStatus data;if(!padRead(0,0,&data)) return in;in.connected=1;
    uint32_t raw=0xffff^data.btns,b=0;
    if(raw&PAD_CROSS) b|=B_JUMP|B_CONFIRM;
    if(raw&PAD_CIRCLE) b|=B_CROUCH|B_BACK;
    if(raw&PAD_R1) b|=B_LIGHT;
    if(raw&(PAD_L1|PAD_L3)) b|=B_RUN;
    if(raw&PAD_TRIANGLE) b|=B_MAP;
    if(raw&PAD_START) b|=B_PAUSE;
    if(raw&PAD_SELECT) b|=B_BACK;
    if(raw&PAD_UP) b|=B_UP;
    if(raw&PAD_DOWN) b|=B_DOWN;
    if(raw&PAD_LEFT) b|=B_LEFT;
    if(raw&PAD_RIGHT) b|=B_RIGHT;
    in.held=b;in.pressed=b&~previous;previous=b;
    // Digital-only pads do not populate the four analog byte fields.
    if((data.mode>>4)==7) {in.move_x=stick(data.ljoy_h);in.move_y=-stick(data.ljoy_v);in.look_x=stick(data.rjoy_h);in.look_y=stick(data.rjoy_v);}
    if(raw&PAD_UP) in.move_y=1;
    if(raw&PAD_DOWN) in.move_y=-1;
    if(raw&PAD_LEFT) in.move_x=-1;
    if(raw&PAD_RIGHT) in.move_x=1;
    return in;
}
static int sample_world(const World *w,int id,int *cursor,int loop,float volume) {
    if(id<0||(unsigned)id>=w->header->sounds)return 0;
    const Sound *s=w->sound+id;const int16_t *pcm=(const int16_t *)(w->blob+s->offset);
    if(s->samples&AUDIO_ULAW)return (int)(voice_ulaw_sample((const uint8_t *)pcm,s->samples&AUDIO_COUNT,cursor,loop)*volume);
    return (int)(voice_sample(pcm,s->samples,cursor,loop)*volume);
}
static int sample(int id,int loop,float volume) {
    return sample_world(&world,id,sound_pos+id,loop,volume);
}
static void audio_pump(void) {
    if(!audio_ready||!world.header||!menu_world.header) return;
    static u64 last_pump;
    u64 now=GetTimerSystemTime();
    if(now-last_pump<kBUSCLK/125)return;
    last_pump=now;
    int menu_audio=game.state==ST_MENU||game.state==ST_HELP||game.state==ST_CREDITS||game.state==ST_NETWORK;
    if(menu_audio!=was_menu_audio) {
        if(menu_audio) {menu_music_pos=0;for(int i=4;i<9;i++)sound_pos[i]=-1;nearby_audible_until=0;nearby_pending=0;}
        else if(game.state==ST_PLAY)for(int i=0;i<4;i++)sound_pos[i]=0;
        printf("STPS MUSIC %s position=%d samples=%lu t=%.6f\n",menu_audio?"START":"STOP",menu_music_pos,
               menu_world.header->sounds?(unsigned long)(menu_world.sound[0].samples&AUDIO_COUNT):0,(double)now/kBUSCLK);
        was_menu_audio=menu_audio;
    }
    int playing=game.state==ST_PLAY||game.state==ST_WON;
    float gains[3];
    for(int i=0;i<3;i++) {
        const float *spec=source_environment_audio[i];gains[i]=spec[3];
        if(spec[6]) {
            Vec3 position=i==2?(Vec3){game.hoover.position[0],game.hoover.position[1],game.hoover.position[2]}:(Vec3){spec[0],spec[1],spec[2]};
            float dx=game.feet.x-position.x,dy=game.feet.y+game.eye-position.y,dz=game.feet.z-position.z;
            float distance=sqrtf(dx*dx+dy*dy+dz*dz);
            gains[i]*=spec[4]/fmaxf(spec[4],fminf(distance,spec[5]));
        }
    }
    // Keep latency below ~90 ms. Never block gameplay waiting for SPU2.
    for(int chunks=0;chunks<4&&audsrv_available()>=(int)sizeof(audio_chunk)&&audsrv_queued()<4096;chunks++) {
        int voice_active=sound_pos[4]>=0;
        if(nearby_pending&&voice_active) {
            int queued=audsrv_queued();if(queued<0)queued=0;
            // PCM is mixed ahead of playback. Include the existing IOP queue
            // so a new look cannot restart the voice during its audible tail.
            nearby_audible_until=GetTimerSystemTime()+((u64)(world.sound[4].samples+queued/2)*kBUSCLK)/22050;
            nearby_pending=0;
        }
        for(int i=0;i<512;i++) {
            int value=0;
            if(menu_audio)value+=sample_world(&menu_world,0,&menu_music_pos,(int)menu_world.config[100],menu_world.config[99]);
            if(playing) {value+=sample(0,1,gains[0]);if(game.moving&&game.grounded) value+=sample(1,1,.30f);
                value+=sample(2,1,gains[1])+sample(3,1,gains[2]);}
            value+=sample(4,0,.8f)+sample(5,0,.8f)+sample(6,0,.5f)+sample(7,0,.65f);
            if(value>32767)value=32767;
            if(value<-32768)value=-32768;
            audio_chunk[i]=(int16_t)value;
        }
        if(enemy_test_enabled&&voice_active&&sound_pos[4]<0){enemy_voice_finished++;printf("STPS ENEMY AUDIO PCM_END count=%d\n",enemy_voice_finished);}
        if(audsrv_play_audio((const char *)audio_chunk,sizeof(audio_chunk))<0) break;
    }
}
int main(int argc,char **argv) {
    int smoke=0,menus=0,models=0,menu_fire=0,benchmark=0,game_only=0,force_pal=0,profile=0,win_test=0,surface_test=0,audio_test=0,bench_focus=-1,walking=0,noonoo_test=0;
    for(int i=1;i<argc;i++) {if(!strcmp(argv[i],"--smoke"))smoke=1;if(!strcmp(argv[i],"--menus"))menus=1;if(!strcmp(argv[i],"--models"))models=1;if(!strcmp(argv[i],"--menufire"))menu_fire=1;if(!strcmp(argv[i],"--bench"))benchmark=1;if(!strcmp(argv[i],"--gameonly"))game_only=1;if(!strcmp(argv[i],"--pal"))force_pal=1;}
    for(int i=1;i<argc;i++)if(!strcmp(argv[i],"--profile"))profile=1;
    for(int i=1;i<argc;i++)if(!strcmp(argv[i],"--enemytest"))enemy_test_enabled=1;
    for(int i=1;i<argc;i++)if(!strcmp(argv[i],"--wintest"))win_test=1;
    for(int i=1;i<argc;i++)if(!strcmp(argv[i],"--surfaces"))surface_test=1;
    for(int i=1;i<argc;i++)if(!strcmp(argv[i],"--audiotest"))audio_test=1;
    for(int i=1;i<argc;i++)if(!strncmp(argv[i],"--benchfocus=",13))bench_focus=atoi(argv[i]+13);
    for(int i=1;i<argc;i++)if(!strcmp(argv[i],"--walkbench"))walking=1;
    for(int i=1;i<argc;i++)if(!strcmp(argv[i],"--noonootest"))noonoo_test=1;
    // Timing probes are optional so normal tests measure the production path.
    profile_enabled=profile;
    net_read_config(argc>0?argv[0]:NULL);
    SifInitRpc(0);while(!SifIopReset("",0)){}while(!SifIopSync()){}
    SifInitRpc(0);SifLoadFileInit();sbv_patch_enable_lmb();
    int pad_ok=module(sio2_start,sio2_end,"freesio2")&&module(pad_start,pad_end,"freepad");
    if(pad_ok) {padInit(0);padPortOpen(0,0,pad_buffer);}
    SifLoadModule("rom0:LIBSD",0,NULL);
    if(module(audsrv_start,audsrv_end,"audsrv")&&audsrv_init()==0) {
        struct audsrv_fmt_t format={22050,16,1};audio_ready=audsrv_set_format(&format)==0;audsrv_set_volume(80);
    }
    for(int i=0;i<9;i++) sound_pos[i]=i<4?0:-1;
    gs=gsKit_init_global_custom(2*1024*1024,64*1024);
    // The release uses 60 Hz so half-rate frames remain nominally 30 FPS,
    // including on a PAL-region console connected to a 60 Hz display.
    gs->Mode=force_pal?GS_MODE_PAL:GS_MODE_NTSC;gs->Width=512;gs->Height=gs->Mode==GS_MODE_PAL?512:448;
    gs->Interlace=GS_INTERLACED;gs->Field=GS_FIELD;gs->PSM=GS_PSM_CT16;gs->PSMZ=GS_PSMZ_16;
    gs->PrimAlphaEnable=GS_SETTING_ON;gs->DoubleBuffering=GS_SETTING_ON;gs->ZBuffering=GS_SETTING_ON;
    // Asset alpha is 0=transparent, 128=opaque. gsKit's default is the reverse.
    gs->PrimAlpha=GS_SETREG_ALPHA(0,1,0,1,0);
    gs->Test->ATE=1;gs->Test->ATST=6;gs->Test->AREF=1;gs->Test->AFAIL=0;
    dmaKit_init(D_CTRL_RELE_OFF,D_CTRL_MFD_OFF,D_CTRL_STS_UNSPEC,D_CTRL_STD_OFF,D_CTRL_RCYC_8,1<<DMA_CHANNEL_GIF);
    dmaKit_chan_init(DMA_CHANNEL_GIF);gsKit_init_screen(gs);gsKit_mode_switch(gs,GS_ONESHOT);gsKit_set_clamp(gs,GS_CMODE_REPEAT);
    int valid=world_open(&world,world_start,world_end-world_start);
    printf("STPS WORLD valid=%d bytes=%ld\n",valid,(long)(world_end-world_start));
    if(!valid) {platform_clear(160,0,0);platform_finish();SleepThread();}
    int menu_valid=world_open(&menu_world,menu_world_start,menu_world_end-menu_world_start);
    printf("STPS MENU WORLD valid=%d bytes=%ld\n",menu_valid,(long)(menu_world_end-menu_world_start));
    if(!menu_valid)SleepThread();
    render_set_menu_world(&menu_world);
    int victory_valid=world_open(&victory_world,victory_world_start,victory_world_end-victory_world_start);
    printf("STPS VICTORY WORLD valid=%d bytes=%ld\n",victory_valid,(long)(victory_world_end-victory_world_start));
    if(!victory_valid)SleepThread();
    render_set_victory_world(&victory_world);
    int walk_valid=render_set_view_animation(player_walk_start,player_walk_end-player_walk_start);
    printf("STPS WALK valid=%d bytes=%ld\n",walk_valid,(long)(player_walk_end-player_walk_start));
    if(!walk_valid)SleepThread();
    printf("STPS READY textures=%lu meshes=%lu trees=%lu vram=%lu pad=%d audio=%d\n",(unsigned long)world.header->textures,(unsigned long)world.header->meshes,(unsigned long)world.header->trees,(unsigned long)gs->CurrentPointer,pad_ok,audio_ready);
    printf("STPS VIDEO %s\n",gs->Mode==GS_MODE_PAL?"PAL":"NTSC");
    game_init(&game);if(smoke||models||benchmark||enemy_test_enabled||win_test||surface_test||audio_test||noonoo_test) {game_start(&game,&world);game.time_of_day=0;}
    if(noonoo_test) {
        game.feet=(Vec3){world.config[C_WAY]-4,.8195f,world.config[C_WAY+2]-5};
        game.yaw=atan2f(4,5);game.pitch=-.15f;game.state_time=6;game.next_scare=10000;
    }
    if(enemy_test_enabled){game.state_time=6;game.next_scare=10000;game.enemy=(Vec3){game.feet.x,game.feet.y,game.feet.z-100};}
    u64 last=GetTimerSystemTime();float elapsed=benchmark&&game_only?18:0;unsigned frames=0;int smoke_stage=0,menu_stage=-1,model_stage=-1;float next_menu_step=0;
    if(bench_focus>=0&&bench_focus<(int)BENCHMARK_COUNT)elapsed=bench_focus*3;
    int bench_stage=-1,enemy_test_started=0;u64 bench_begin=0;unsigned bench_frames=0;
    int win_stage=0,win_last_state=-1;float win_notice_begin=0,win_scene_begin=0;
    int surface_stage=-1,surface_crossed=0;float surface_minimum=100;
    int audio_stage=-1,flash_switches=0,reserved_ignored=0;
    float hoover_angle=0,hoover_turn=0,hoover_minx=1e9f,hoover_maxx=-1e9f,hoover_minz=1e9f,hoover_maxz=-1e9f;
    int walk_stage=-1;u64 walk_begin_time=0;unsigned walk_frames=0,walk_slow=0;float walk_distance=0,walk_worst=0;
    if(walking&&bench_focus>=0&&bench_focus<(int)WALK_COUNT)elapsed=bench_focus*12;
    for(;;) {
        u64 now=GetTimerSystemTime();float dt=(float)(now-last)/(float)kBUSCLK;last=now;
        Input in=pad_ok?read_input():(Input){0};
        if(noonoo_test){in=(Input){.connected=1};game.enemy=(Vec3){0,0,0};}
        if(walking) {
            unsigned route=(unsigned)(elapsed/12);float phase=elapsed-route*12;
            if((int)route!=walk_stage) {
                if(walk_stage>=0){float seconds=(float)(now-walk_begin_time)/kBUSCLK;
                    printf("STPS BENCH scene=%s frames=%u seconds=%.3f fps=%.2f\n",walk_names[walk_stage],walk_frames,seconds,walk_frames/seconds);
                    printf("STPS WALK route=%s distance=%.2f slow=%u total=%u worst_ms=%.2f state=%d\n",walk_names[walk_stage],walk_distance,walk_slow,walk_frames,walk_worst,game.state);
                    if(walk_distance<8||game.state!=ST_PLAY){printf("STPS SMOKE FAIL walk-progress\n");SleepThread();}}
                if(route>=WALK_COUNT||(bench_focus>=0&&(int)route>bench_focus)){printf("STPS SMOKE PASS walk\n");SleepThread();}
                walk_stage=route;walk_begin_time=now;walk_frames=walk_slow=0;walk_distance=walk_worst=0;walk_begin(&game,&world,route);
                printf("STPS WALK START route=%s frame=%u\n",walk_names[route],frames);
            }
            in=walk_input(route,phase);
            if(phase>=1){if(!walk_frames)walk_begin_time=now;walk_frames++;if(dt>.036f)walk_slow++;walk_worst=fmaxf(walk_worst,dt*1000);}
        }
        if(audio_test) {
            int stage=(int)(elapsed/3);in=(Input){.connected=1};game.state_time=6;game.next_scare=10000;
            if(stage!=audio_stage) {
                audio_stage=stage;
                if(stage==1||stage==2)in.pressed=B_LIGHT;
                if(stage==3||stage==4)in.pressed=4096; // Retired input must have no effect.
                if(stage==5){game.feet.x=world.config[C_DOOR];game.feet.z=727;game.feet.y=world_height(&world,game.feet.x,game.feet.z);}
                if(stage==6){game.feet=(Vec3){world.config[C_SPAWN],world.config[C_SPAWN+1],world.config[C_SPAWN+2]};}
                printf("STPS AUDIO step=%d t=%.3f\n",stage,elapsed);
            }
            game.enemy=(Vec3){game.feet.x,game.feet.y,game.feet.z-100};
        }
        if(surface_test) {
            int stage=elapsed<28?(int)(elapsed/7):4+(int)((elapsed-28)/2);
            float phase=elapsed-(stage<4?stage*7:28+(stage-4)*2);
            if(stage!=surface_stage) {
                if(surface_stage>=0&&surface_stage<4) {
                    float exit=hypotf(game.feet.x-source_roof_support[0],game.feet.z-source_roof_support[1]);
                    printf("STPS ROOF route=%d centre=%d minimum=%.3f exit=%.3f\n",surface_stage,surface_crossed,surface_minimum,exit);
                    if(!surface_crossed||surface_minimum<6.5f||exit<5){printf("STPS SMOKE FAIL roof\n");SleepThread();}
                }
                if(stage>=7){printf("STPS SMOKE PASS surfaces frames=%u\n",frames);SleepThread();}
                surface_stage=stage;surface_crossed=0;surface_minimum=100;
                game_start(&game,&world);game.time_of_day=0;game.state_time=6;game.next_scare=10000;
                if(stage<4) {
                    game.yaw=stage*1.5707963f;game.pitch=-.35f;
                    game.feet=(Vec3){source_roof_support[0]-sinf(game.yaw)*18,12,source_roof_support[1]-cosf(game.yaw)*18};game.grounded=0;
                } else if(stage==4){game.pitch=-1.0472f;game.draw_grass=0;}
                else {const float *p=world.config+C_CUSTARD+27;game.feet=(Vec3){p[0]+20,world_height(&world,p[0]+20,p[2]-8),p[2]-8};game.pitch=-.35f;game.yaw=stage==6?.55f:0;}
                game.enemy=(Vec3){game.feet.x,game.feet.y,game.feet.z-100};
                printf("STPS SURFACE START scene=%s frame=%u\n",stage==4?"body-idle":stage==5?"grass-front":stage==6?"grass-side":"roof",frames);
            }
            in=(Input){.connected=1,.move_y=stage<4&&phase>.9f?1:0};
        }
        if(win_test) {
            in=(Input){.connected=1};game.next_scare=10000;
            if(win_stage==0&&elapsed>=1) {
                game.custards=511;game.collected=9;const float *p=world.config+C_CUSTARD+27;
                game.feet=(Vec3){p[0],world_height(&world,p[0],p[2]),p[2]};
                game.enemy=(Vec3){p[0],game.feet.y,p[2]-100};win_stage=1;win_notice_begin=elapsed;
            }
            if(win_stage==2&&elapsed-win_scene_begin>=2.5f)in.pressed=B_CONFIRM;
        }
        if(enemy_test_enabled) {
            in=(Input){.connected=1};
            if(elapsed>=1&&elapsed<3){game.enemy=(Vec3){game.feet.x,game.feet.y+.675f,game.feet.z+15};game.yaw=0;game.pitch=0;}
            if(elapsed>=3) {
                if(!enemy_test_started){
                    if(game.health!=100||enemy_side_shifts||game.enemy_attacking){printf("STPS SMOKE FAIL distant-damage health=%.3f shifts=%d\n",game.health,enemy_side_shifts);SleepThread();}
                    printf("STPS ENEMY DISTANT PASS health=%.1f shifts=%d\n",game.health,enemy_side_shifts);
                    enemy_test_started=1;game.enemy=(Vec3){game.feet.x,game.feet.y+.675f,game.feet.z+4.5f};game.enemy_visible=0;
                }
                float phase=elapsed-3;
                float duration=world.sound[4].samples/22050.f;
                int looking=phase<.12f||(phase>=.2f&&phase<.35f)||(phase>=2&&phase<2.15f)||(phase>=5&&phase<5.15f)||
                    (phase>=duration+.7f&&phase<duration+.9f)||(phase>=duration+1.1f&&phase<duration+1.25f);
                game.yaw=atan2f(game.enemy.x-game.feet.x,game.enemy.z-game.feet.z)+(looking?0:3.1415927f);
            }
        }
        if(menu_fire)in=(Input){.connected=1};
        if(benchmark) {
            unsigned scene=(unsigned)(elapsed/3);
            if((int)scene!=bench_stage) {
                if(bench_stage>=0){float seconds=(float)(now-bench_begin)/kBUSCLK;
                    printf("STPS BENCH scene=%s frames=%u seconds=%.3f fps=%.2f\n",benchmark_names[bench_stage],bench_frames,seconds,bench_frames/seconds);}
                if(scene>=BENCHMARK_COUNT||(bench_focus>=0&&(int)scene>bench_focus)){printf("STPS SMOKE PASS benchmark\n");SleepThread();}
                bench_stage=scene;bench_begin=0;bench_frames=0;
                printf("STPS BENCH START scene=%s frame=%u\n",benchmark_names[scene],frames);
            }
            in=(Input){.connected=1};benchmark_fixture(&game,&world,scene,elapsed-scene*3);
            if(elapsed-scene*3>=.5f){if(!bench_begin)bench_begin=now;bench_frames++;}
        }
        int expect=-1;
        if(menus) {
            in=(Input){.connected=1};
            if(elapsed>=next_menu_step && menu_stage+1<(int)MENU_ROUTE_COUNT) {
                menu_stage++;next_menu_step=elapsed+.8f;
                const MenuStep *s=menu_route+menu_stage;
                in.pressed=s->button;expect=s->state;menu_fixture(&game,&world,s->fixture);
            }
        }
        if(smoke) {in=(Input){.connected=1};
            if(smoke_stage==0) in.look_x=.3f;
            if(elapsed>3&&smoke_stage==0) {game.feet.x=world.config[C_DOOR];game.feet.z=727;game.feet.y=world_height(&world,game.feet.x,game.feet.z);game.yaw=0;smoke_stage=1;printf("STPS SMOKE HOUSE\n");}
            if(smoke_stage==1) {in.move_y=1;if(game.feet.z>741) {smoke_stage=2;printf("STPS SMOKE ENTERED\n");}}
            if(smoke_stage==2) {in.move_y=-1;if(game.feet.z<729) {smoke_stage=3;game.time_of_day=2;printf("STPS SMOKE EXITED NIGHT\n");}}
        }
        if(models) {
            in=(Input){.connected=1};int stage=(int)(elapsed/2);if(stage>5)stage=5;
            game.state_time=6;
            game.time_of_day=stage<2?2:0;game.flashlight=stage!=1;game.pitch=stage<2?-.5f:.18f;
            game.feet=(Vec3){world.config[C_SPAWN],world.config[C_SPAWN+1],world.config[C_SPAWN+2]};game.yaw=0;
            if(stage>=2) {
                int kind=stage<=3?0:stage-3;const Tree *chosen=NULL;
                for(unsigned i=0;i<world.header->trees;i++)if((int)world.tree[i].type==kind) {
                    if(!chosen||world.tree[i].y<chosen->y)chosen=world.tree+i;
                }
                if(chosen){game.feet.x=chosen->x+(stage==3?10:0);game.feet.z=chosen->z-(stage==3?0:10);game.yaw=stage==3?-1.5707963f:0;}
            }
            game.feet.y=world_height(&world,game.feet.x,game.feet.z);game.vy=0;game.health=100;game.next_scare=10000;
            game.enemy=(Vec3){game.feet.x,game.feet.y,game.feet.z-100};
            if(stage!=model_stage){model_stage=stage;printf("STPS MODEL step=%d pitch=%.2f light=%d\n",stage,game.pitch,game.flashlight);}
        }
        if(!in.connected&&game.state==ST_PLAY) {game.state=ST_PAUSE;game.moving=0;}
        u64 update_start=GetTimerSystemTime();
        Vec3 previous_feet=game.feet;
        game_advance(&game,&world,&in,dt);net_frame(&game,&world,dt);
        if(walking)walk_distance+=hypotf(game.feet.x-previous_feet.x,game.feet.z-previous_feet.z);
        if(surface_test&&surface_stage<4) {
            float radius=hypotf(game.feet.x-source_roof_support[0],game.feet.z-source_roof_support[1]);
            if(radius<18)surface_minimum=fminf(surface_minimum,game.feet.y);
            if(radius<5){surface_crossed=1;if(game.feet.y<10.8f){printf("STPS SMOKE FAIL roof-centre\n");SleepThread();}}
        }
        if(win_test&&game.state!=win_last_state) {
            printf("STPS WIN STATE state=%d frame=%u t=%.3f count=%d\n",game.state,frames,elapsed,game.collected);
            win_last_state=game.state;
            if(game.state==ST_WINSCENE){win_stage=2;win_scene_begin=elapsed;
                if(elapsed-win_notice_begin<4.95f){printf("STPS SMOKE FAIL win-duration\n");SleepThread();}}
            if(win_stage==2&&game.state==ST_MENU)win_stage=3;
        }
        static u64 update_ticks;update_ticks+=GetTimerSystemTime()-update_start;
        if(profile_enabled&&frames%120==119){printf("STPS UPDATE %.2f ms\n",update_ticks*(1000.f/(kBUSCLK*120.f)));update_ticks=0;}
        if(expect>=0) {
            printf("STPS MENU step=%d expected=%d actual=%d day=%d map=%d light=%d\n",menu_stage,expect,game.state,game.time_of_day,game.map,game.flashlight);
            if(!menu_step_ok(&game,menu_stage)) {printf("STPS SMOKE FAIL menu=%d\n",menu_stage);SleepThread();}
        }
        if(game.events&EV_SWITCH) sound_pos[6]=0;
        if(audio_test&&in.pressed==4096) {
            if(game.events){printf("STPS SMOKE FAIL retired-input\n");SleepThread();}
            reserved_ignored++;
        }
        if(audio_test&&(game.events&EV_SWITCH)) {
            flash_switches++;
            printf("STPS AUDIO SWITCH flashlight=%d t=%.3f\n",flash_switches,elapsed);
        }
        if(game.events&EV_SCARE)sound_pos[7]=0;
        if(game.events&EV_ENEMY_SEEN) {
            int started=voice_start_ready(sound_pos+4,world.sound[4].samples,GetTimerSystemTime(),nearby_audible_until);
            if(started)nearby_pending=1;
            if(enemy_test_enabled) {
                if(started)enemy_voice_starts++;else enemy_voice_ignored++;
                printf("STPS ENEMY AUDIO %s t=%.3f cursor=%d\n",started?"START":"BUSY",elapsed,sound_pos[4]);
            }
        }
        if(enemy_test_enabled&&(game.events&EV_ENEMY_SHIFT)){enemy_side_shifts++;printf("STPS ENEMY SIDE t=%.3f pos=%.3f,%.3f,%.3f\n",elapsed,game.enemy.x,game.enemy.y,game.enemy.z);}
        if(game.events&EV_DEATH) {sound_pos[4]=-1;nearby_pending=0;sound_pos[5]=0;}
        if(noonoo_test) {
            float m[16];game_hoover_matrix(&world,&game,m);float angle=atan2f(m[8],m[10]);
            hoover_turn+=fabsf(remainderf(angle-hoover_angle,6.2831853f));hoover_angle=angle;
            hoover_minx=fminf(hoover_minx,game.hoover.position[0]);hoover_maxx=fmaxf(hoover_maxx,game.hoover.position[0]);
            hoover_minz=fminf(hoover_minz,game.hoover.position[2]);hoover_maxz=fmaxf(hoover_maxz,game.hoover.position[2]);
            if(frames%30==0)printf("STPS NOONOO frame=%u pos=%.3f,%.3f,%.3f forward=%.3f,%.3f,%.3f\n",frames,
                game.hoover.position[0],game.hoover.position[1],game.hoover.position[2],m[8],m[9],m[10]);
        }
        audio_pump();render_frame(&world,&game,in.connected);elapsed+=dt;frames++;
        if(noonoo_test&&elapsed>8) {
            int pass=hoover_turn>6.2831853f&&hoover_maxx-hoover_minx>1&&hoover_maxz-hoover_minz>1;
            printf("STPS NOONOO SUMMARY turns=%.3f x_span=%.3f z_span=%.3f\n",hoover_turn/6.2831853f,hoover_maxx-hoover_minx,hoover_maxz-hoover_minz);
            printf("STPS SMOKE %s noonoo frames=%u\n",pass?"PASS":"FAIL",frames);SleepThread();
        }
        if(audio_test&&elapsed>21) {
            int pass=flash_switches==2&&reserved_ignored==2&&game.flashlight&&game.state==ST_PLAY;
            printf("STPS SMOKE %s audio frames=%u flashlight=%d reserved_ignored=%d\n",pass?"PASS":"FAIL",frames,flash_switches,reserved_ignored);SleepThread();
        }
        if(win_test&&win_stage==3) {printf("STPS SMOKE PASS victory frames=%u notice=%.3f final=%.3f\n",frames,win_scene_begin-win_notice_begin,elapsed-win_scene_begin);SleepThread();}
        if(frames%120==0) printf("STPS FRAME %u t=%.2f state=%d pos=%.2f,%.2f,%.2f health=%.1f\n",frames,elapsed,game.state,game.feet.x,game.feet.y,game.feet.z,game.health);
        if(enemy_test_enabled&&elapsed>world.sound[4].samples/22050.f*2+5) {
            int pass=enemy_voice_starts==2&&enemy_voice_ignored>=2&&enemy_voice_finished==2&&enemy_side_shifts>=2&&game.state==ST_PLAY&&game.health>0;
            printf("STPS ENEMY SUMMARY starts=%d ignored=%d finished=%d shifts=%d health=%.1f\n",enemy_voice_starts,enemy_voice_ignored,enemy_voice_finished,enemy_side_shifts,game.health);
            printf("STPS SMOKE %s enemy frames=%u\n",pass?"PASS":"FAIL",frames);SleepThread();
        }
        if(smoke&&elapsed>12) {printf("STPS SMOKE %s frames=%u stage=%d\n",smoke_stage==3?"PASS":"FAIL",frames,smoke_stage);SleepThread();}
        if(models&&elapsed>12) {printf("STPS SMOKE PASS models frames=%u\n",frames);SleepThread();}
        if(menu_fire&&elapsed>6) {printf("STPS SMOKE PASS menufire frames=%u menu_time=%.2f\n",frames,game.state_time);SleepThread();}
        if(menus&&menu_stage+1==(int)MENU_ROUTE_COUNT&&elapsed>=next_menu_step) {printf("STPS SMOKE PASS menus frames=%u\n",frames);SleepThread();}
    }
}

