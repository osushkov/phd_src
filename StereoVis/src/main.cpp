
#include <cassert>
#include <iostream>
#include <SDL/SDL.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <vector>
#include <cmath>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include <netdb.h>

#include "Graphics/GraphicsBase.h"
#include "Settings.h"

#define GRID_WIDTH 64
#define GRID_DEPTH 48
#define GRID_SPACING 1.0f

#define REMOTE_STEREO_IP "localhost"
#define REMOTE_STEREO_PORT 6000


struct DepthFeature {
    float lpixel_x, lpixel_y;
    float rpixel_x, rpixel_y;
    float xpos, ypos, zpos;
};


bool done = false;
int socket_fd = 0;

float grid_heights[GRID_WIDTH][GRID_DEPTH];
std::vector<DepthFeature> depth_features;
float cur_floor = 85.0f;


float getHeight(unsigned x, unsigned z){
    float grid_xpos = (2.0f*(float)x/GRID_WIDTH - 1.0f)*30.0f;
    float grid_ypos = (2.0f*(float)z/GRID_DEPTH - 1.0f)*30.0f;
    
    float weight_sum = 0.0f;
    float sigma = 1.0f;
    for(unsigned i = 0; i < depth_features.size(); i++){
        float dist2 = (grid_xpos-depth_features[i].xpos)*(grid_xpos-depth_features[i].xpos) + 
                      (grid_ypos-depth_features[i].ypos)*(grid_ypos-depth_features[i].ypos);
       
        weight_sum += expf(-dist2/(2.0f*sigma*sigma));
    }

    if(weight_sum < 0.1f){ return cur_floor; }

    float result = 0.0f;
    for(unsigned i = 0; i < depth_features.size(); i++){
        float dist2 = (grid_xpos-depth_features[i].xpos)*(grid_xpos-depth_features[i].xpos) + 
                      (grid_ypos-depth_features[i].ypos)*(grid_ypos-depth_features[i].ypos);
        result += depth_features[i].zpos * expf(-dist2/(2.0f*sigma*sigma))/weight_sum;
    }

    
    //std::cout << result << std::endl;
    return result;
}

bool sanityCheck(DepthFeature feature){
    return feature.xpos < 30.0f && feature.xpos > -30.0f &&
           feature.ypos < 30.0f && feature.ypos > -30.0f &&
           feature.zpos < 200.0f && feature.zpos > 10.0f;
}

bool isSuitableFloor(float floor){
    unsigned num_nearby = 0;
    for(unsigned i = 0; i < depth_features.size(); i++){
        if(fabs(depth_features[i].zpos - floor) < 2.0f){
            num_nearby++;
        }
    }
    return num_nearby >= 10;
}

float findCurFloor(void){
    float furthest = 0.0f;
    bool is_set = false;

    for(unsigned i = 0; i < depth_features.size(); i++){
        if((!is_set || depth_features[i].zpos > furthest) && isSuitableFloor(depth_features[i].zpos)){
            is_set = true;
            furthest = depth_features[i].zpos;
        }
    }
    return furthest;
}

bool update(void){
    struct sockaddr_in serv_addr;
    struct hostent *he;

    if((socket_fd = socket(PF_INET, SOCK_STREAM, 0)) == -1){
        return false;
    }

    if((he = gethostbyname(REMOTE_STEREO_IP)) == NULL) {
        return false;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(REMOTE_STEREO_PORT);
	serv_addr.sin_addr = *((struct in_addr*) he->h_addr);
	memset(serv_addr.sin_zero, '\0', sizeof(serv_addr.sin_zero));
	if((connect(socket_fd, (struct sockaddr*) &serv_addr, sizeof(serv_addr))) == -1) {
		return false;
	}
    
    unsigned num_features = 0;
	if(recv(socket_fd, &num_features, sizeof(unsigned), 0) < 1) { return false; }
    
    depth_features.clear();
    for(unsigned i = 0; i < num_features; i++){
        DepthFeature cur_feature;
        unsigned num_received = 0;
        while(num_received < sizeof(DepthFeature)){
            num_received += recv(socket_fd, &cur_feature + num_received, sizeof(DepthFeature)-num_received, 0);
        }
        if(sanityCheck(cur_feature)){
            depth_features.push_back(cur_feature);
        }
        else{
            std::cout << "sanity!\n" << std::endl;
        }
    }

    close(socket_fd);
  
    std::cout << "num depth features: " << depth_features.size() << std::endl;
    cur_floor = 0.9f*cur_floor + 0.1f*findCurFloor();
    std::cout << "floor: " << cur_floor << std::endl;
    for(unsigned z = 0; z < GRID_DEPTH; z++){
        for(unsigned x = 0; x < GRID_WIDTH; x++){
            grid_heights[x][z] = getHeight(x, z);// + 60.0f)/2.0f;
        }
    }


    return true;
}

void render(void){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glPolygonMode(GL_POINT, GL_FRONT_AND_BACK);

    glPushMatrix();
    //glRotatef(-30.0f, 1.0f, 0.0f, 0.0f);
    glTranslatef(0.0f, 0.0f, -60.0f);
    
    // Draw the ground
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    glDisable(GL_TEXTURE_2D);
 
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    
    glTranslatef(-GRID_SPACING * GRID_WIDTH/2.0f, 0.0f, -GRID_SPACING*GRID_DEPTH/2.0f);
   /*
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D); 
    for(unsigned i = 0; i < depth_features.size(); i++){
        glPushMatrix();
        glTranslatef(depth_features[i].xpos, (depth_features[i].zpos-50.0f)/10.0f, depth_features[i].ypos);
        glutSolidSphere(1.0f, 5, 5);
        glPopMatrix();
    }*/

    for(unsigned z = 0; z < GRID_DEPTH-1; z++){
        glBegin(GL_LINE_STRIP);
        for(unsigned x = 0; x < GRID_WIDTH; x++){
            float hc = -(grid_heights[x][z]-cur_floor)/20.0f;

            glColor3f(hc, 1.0f-hc, 0.0f);   
            glVertex3f(x*GRID_SPACING, 0.0f, z*GRID_SPACING);
            glVertex3f(x*GRID_SPACING, 0.0f, (z+1)*GRID_SPACING);
        }
        glEnd();
    }
    glPopMatrix();

    //glBegin(GL_POINTS


    SDL_GL_SwapBuffers( );
}


void pollEvents(void) {
    SDL_Event event;
    while(SDL_PollEvent(&event)){
        switch(event.type){
            case SDL_QUIT:
                done = true;
                break;
            case SDL_KEYUP:
                if(event.key.keysym.sym == SDLK_ESCAPE){ done = true; }
                break;
            default:
                break;
        }
    }
}




int main(int argc, char **argv){
    if (!Settings::instance().load("data/settings.cfg")) {
        std::cerr << "Could not load settings file properly" << std::endl;
        return 1;
    }

    glutInit(&argc, argv);
    GraphicsBase::initialise();

    while (!done) {
        pollEvents();
        update();
        render();
    }

    SDL_Quit();

    return 0;
}

