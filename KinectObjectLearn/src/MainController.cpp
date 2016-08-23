
#include "MainController.h"

#include "KinectCamera/KinectCamera.h"
#include "KinectCamera/RemoteKinectCamera.h"
#include "KinectCamera/PlaybackKinectCamera.h"
#include "KinectCamera/Recorder.h"
#include "Features/SIFT/sift.h"
#include "Features/SIFT/imgfeatures.h"
#include "Util/Common.h"
#include "Util/ParallelServer.h"
#include "Util/Timer.h"
#include "Util/PerfStats.h"
#include "Util/Semaphore.h"
#include "Util/Geometry.h"
#include "Util/Octree.h"
#include "Util/Octree.hpp"
#include "Util/ConvexHull.h"
#include "ObjectLearner/TraceTracker.h"
#include "ObjectLearner/FrameHistoryBuffer.h"
#include "ObjectFinder.h"
#include "Arm/Arm.h"
#include "Settings.h"
#include "Control.h"
#include "ObjectLearner/ObjectLearner.h"
#include "Console.h"
#include "Arm/Arm.h"
#include "Arm/ArmForwardKinematics.h"
#include "Visualisation/SceneRenderer.h"
#include "Visualisation/CylinderRenderObject.h"
#include "Visualisation/PointCloudRenderObject.h"
#include "Visualisation/LinkedPointsRenderObject.h"
#include "Visualisation/SuperQuadricRenderObject.h"
#include "Reconstruction/ReconstructionManager.h"
#include "Reconstruction/SuperQuadric.h"
#include "Reconstruction/SuperQuadricBestFit.h"
#include "Reconstruction/ICPMatch.h"
#include "Reconstruction/CentralisePoints.h"
#include "Object.h"
#include "CameraController.h"
#include "OptimiserClass/Common.h"
#include "Match.h"

#include <algorithm>

MainController& MainController::instance(void){
    static MainController main_controller;
    return main_controller;
}

MainController::MainController() {

}


void MainController::initialise(void){

}

void MainController::reload(void){

    //initialise();
}

void MainController::learnObject(std::string object_name){
    Object object(object_name);
    object.buildStage(OBJECT_BS_FULL);
}

void MainController::examineObject(std::string object_name){
	Object object(object_name);
	object.examine();
}

void MainController::generateSIFT(std::string vid_path, std::string out_path){
    std::cout << "MainController: generateSIFT - " 
              << vid_path << " : " 
              << out_path << std::endl;

    CameraController camera_controller(vid_path, true);
    camera_controller.createViewWindows();

    std::fstream out_sift(out_path.c_str(), std::ios::out | std::ios::binary);

    running = true;
    while (camera_controller.getNewFrame() && running) {
        KinectCamera::CorrelatedImage correlated_frame = camera_controller.getCorrelatedFrame();
        IplImage* img = Common::imageFromCorrelatedFrame(correlated_frame);

        std::vector<SIFTFeature3D> frame_features;
        Common::extractFeature(correlated_frame, frame_features);

        unsigned num_features = frame_features.size();
        out_sift.write((const char *)&num_features, sizeof(unsigned));

        for(unsigned i = 0; i < frame_features.size(); i++){
            write_feature(out_sift, frame_features[i].sift_feature);
            out_sift.write((const char*)&(frame_features[i].position), sizeof(Vector3D));

            draw_features(img, &(frame_features[i].sift_feature), 1);
        }

        camera_controller.showImageAndContinue(img);
        cvReleaseImage(&img);
    }

    camera_controller.closeWindow("RGB");
}

void MainController::reconstruct(std::string object_name){
    std::stringstream rsnap_str;
    rsnap_str << "data/" << object_name << "_rsnapshots.rsnap";

    SnapshotFrames sframes;
    if(!sframes.load(rsnap_str.str())){
        std::cout << "error" << std::endl;
    }

    std::vector<SnapshotFrame> frames = sframes.getFrames();
    for(unsigned i = 0; i < frames.size(); i++){
        frames[i].to_camera.print();
    }

    ReconstructionManager rmanager(object_name);
    std::vector<SnapshotFrame> mframes = rmanager.reconstruct(frames);

    std::stringstream mf_str;
    mf_str << "data/" << object_name << "_frames.mf";

    std::ofstream out_file(mf_str.str().c_str(), std::ios::binary);
    unsigned num_frames = mframes.size();
    out_file.write((char*)&num_frames, sizeof(unsigned));
    for(unsigned i = 0; i < mframes.size(); i++){
        mframes[i].writeOutFrame(out_file);
    }
}

static std::string getOutputFilename(){
    std::stringstream str;
    static unsigned num;
    str << "data/output/output" << num++ << ".cframe";
    return str.str();
}

static void saveFrame(KinectCamera::CorrelatedImage &cframe){
    std::string filename = getOutputFilename();
    std::cout << "saving cframe to: " << filename << std::endl;

    std::ofstream out_file(filename.c_str(), std::ios::out|std::ios::binary);
    out_file.write((char*)&(cframe.width), sizeof(unsigned));
    out_file.write((char*)&(cframe.height), sizeof(unsigned));

    unsigned num_pixels = cframe.depth_pixels.size();
    out_file.write((char*)&num_pixels, sizeof(unsigned));

    for(unsigned i = 0; i < num_pixels; i++){
        out_file.write((char*)&(cframe.depth_pixels[i].color), sizeof(Vector3D));
        out_file.write((char*)&(cframe.depth_pixels[i].pos), sizeof(Vector3D));
        out_file.write((char*)&(cframe.depth_pixels[i].have_pos), sizeof(bool));
    }
}

static KinectCamera::CorrelatedImage loadFrame(std::string filename){
    KinectCamera::CorrelatedImage cframe;

    std::ifstream in_file(filename.c_str(), std::ios::in|std::ios::binary);
    in_file.read((char*)&(cframe.width), sizeof(unsigned));
    in_file.read((char*)&(cframe.height), sizeof(unsigned));

    unsigned num_pixels;
    in_file.read((char*)&num_pixels, sizeof(unsigned));

    for(unsigned i = 0; i < num_pixels; i++){
        KinectCamera::DepthPixel new_pixel;

        in_file.read((char*)&(new_pixel.color), sizeof(Vector3D));
        in_file.read((char*)&(new_pixel.pos), sizeof(Vector3D));
        in_file.read((char*)&(new_pixel.have_pos), sizeof(bool));

        cframe.depth_pixels.push_back(new_pixel);
    }

    return cframe;
}

void MainController::test(void){
    SceneRenderer::instance().initialise();

    PointCloudRenderObject *point_cloud = new PointCloudRenderObject(false, Vector3D(0.0f,0.0f,0.0f));
    SceneRenderer::instance().addObject(point_cloud);

    std::string filename = "data/truck.final";
    std::ifstream object_shape_file(filename.c_str(), std::ios::in|std::ios::binary);

    unsigned num_frames = 0;
    object_shape_file.read((char*)&num_frames, sizeof(unsigned));

    std::vector<SnapshotFrame> object_frames;
    for(unsigned i = 0; i < num_frames; i++){
        SnapshotFrame new_frame = SnapshotFrame::readFrameFeatures(object_shape_file);
        object_frames.push_back(new_frame);
    }

    std::vector<PointCloudPoint> point_cloud_points;
    unsigned num_surface_points;
    object_shape_file.read((char*)&num_surface_points, sizeof(unsigned));
    for(unsigned i = 0; i < num_surface_points; i++){
        Vector3D new_surface_point;
        object_shape_file.read((char*)&new_surface_point, sizeof(Vector3D));

        PointCloudPoint new_point;
        new_point.pos = new_surface_point;
        point_cloud_points.push_back(new_point);
    }

    point_cloud->clearPoints();
    point_cloud->addPoints(point_cloud_points);

    /*
    CameraController camera_controller(false);
    camera_controller.createViewWindows();

    const Transform t = Common::cameraToWorldSpaceTransform(0.0f, 25.0f);
    while (camera_controller.getNewFrame()) {

        KinectCamera::CorrelatedImage cframe = camera_controller.getCorrelatedFrame();

        for(unsigned i = 0; i < cframe.depth_pixels.size(); i++){
            if(cframe.depth_pixels[i].have_pos){
                cframe.depth_pixels[i].pos = t.mat*cframe.depth_pixels[i].pos + t.shift;
            }
        }

        std::vector<PointCloudPoint> point_cloud_points;
        for(unsigned i = 0; i < cframe.depth_pixels.size(); i++){
            if(cframe.depth_pixels[i].have_pos){
                PointCloudPoint new_point;
                new_point.pos = cframe.depth_pixels[i].pos;
                new_point.color = cframe.depth_pixels[i].color;
                new_point.color.scale(1.0f/255.0f);
                point_cloud_points.push_back(new_point);
            }
        }
        point_cloud->clearPoints();
        point_cloud->addPoints(point_cloud_points);


        IplImage *img = Common::imageFromCorrelatedFrame(cframe);
        char r = camera_controller.showImageAndContinue(img);
        if(r == 'q'){
            cvReleaseImage(&img);
            break;
        }
        if(r == 's'){
            saveFrame(cframe);
        }

        cvReleaseImage(&img);
    }
    */
}

void MainController::generateMovement(std::string record_path){

    /*std::vector<float> euler_angles;
    euler_angles.push_back(136.0f);
    euler_angles.push_back(45.0f);
    euler_angles.push_back(130.0f);
    std::vector<Vector3D> gripper_orientation = Control::gripperEulerAnglesToOrientation(euler_angles);

    for(unsigned i = 0; i < gripper_orientation.size(); i++){
        gripper_orientation[i].print();
    }
    return;*/

    
    Arm::getArm()->closeHand(0.3f);
    getchar();
    Common::sleep(5000);
    Arm::getArm()->closeHand(0.71f);
    getchar();

    std::vector<Vector3D> positions;
    //positions.push_back(Vector3D(286.0f, 314.0f, 243.0f));
    positions.push_back(Vector3D(330.0f, 314.0f, 400.0f));
    positions.push_back(Vector3D(360.0f, 314.0f, 165.0f));
    positions.push_back(Vector3D(472.0f, 314.0f, 285.0f));
    //positions.push_back(Vector3D(354.0f, 314.0f, 343.0f));
    positions.push_back(Vector3D(250.0f, 314.0f, 243.0f));
    //positions.push_back(Vector3D(299.0f, 314.0f, 415.0f));


    std::vector<Vector3D> orientation;
    orientation.push_back(Vector3D(0.0f, 1.0f, 0.0f));
    orientation.push_back(Vector3D(0.0f, 0.0f, 1.0f));
    orientation.push_back(Vector3D(1.0f, 0.0f, 0.0f));
    std::vector<Vector3D> fix_mat = Geometry::axisRotationMatrix(orientation[2], -60.0f*M_PI/180.0f);
    for(unsigned i = 0; i < orientation.size(); i++){
        orientation[i].matrixMultLeft(fix_mat);
    }

    std::vector<Vector3D> pose;
    pose.push_back(positions[0]);
    pose.push_back(orientation[0]);
    pose.push_back(orientation[1]);
    pose.push_back(orientation[2]);

    float rot_offset = 15.0f;

    KinectCamera *cam = KinectCamera::getLiveCamera();
    Recorder *recorder = NULL;

    std::cout << "start?" << std::endl;
    getchar();

    if(record_path.size() > 0){
        recorder = new Recorder(record_path, cam, Arm::getArm());
        recorder->startRecording(0);
    }
    else{
        std::cout << "Not recording" << std::endl;
    }

    for(unsigned iter = 0; iter < 6; iter++){
        for(unsigned i = 0; i < positions.size(); i++){
            std::cout << "Position index: " << i << std::endl;
            pose[0] = positions[i%positions.size()];
            Control::moveArmToWorldPoint(pose);
            

            fix_mat = Geometry::axisRotationMatrix(pose[3], rot_offset*M_PI/180.0f);
            for(unsigned j = 1; j < pose.size(); j++){
                pose[j].matrixMultLeft(fix_mat);
            }
        }
    }

    if(recorder != NULL){
        recorder->stopRecording();
        delete recorder;
    }

    delete cam;
}

void MainController::rotate(std::string record_path){
    Arm::getArm()->closeHand(0.0f);
    Common::sleep(5000);
    Arm::getArm()->closeHand(0.9f);
    getchar();

    // TODO: move the hand into a position suitable for observation.

    Arm::getArm()->moveJointAbs(6, -180.0f);
    while(fabs(Arm::getArm()->requestCurrentAngles().at(5) + 180.0f) > 5.0f){
        Common::sleep(100);
    }

    KinectCamera *cam = KinectCamera::getLiveCamera();
    Recorder *recorder = NULL;

    if(record_path.size() > 0){
        recorder = new Recorder(record_path, cam);
        recorder->startRecording();
    }

    Arm::getArm()->moveJointAbs(6, 180.0f);
    while(fabs(Arm::getArm()->requestCurrentAngles().at(5) - 180.0f) > 5.0f){
        std::cout << Arm::getArm()->requestCurrentAngles().at(5) << std::endl;
        Common::sleep(100);
    }

    getchar();

    if(recorder != NULL){
        recorder->stopRecording();
        delete recorder;
    }

    delete cam;
}

void MainController::sceneMatchObject(std::string object_name){
    std::cout << "MainController: sceneMatchObject " << object_name << std::endl;

    SceneRenderer::instance().initialise();

    PointCloudRenderObject *point_cloud = new PointCloudRenderObject(false, Vector3D(0.0f,1.0f,0.0f));
    SceneRenderer::instance().addObject(point_cloud);

    Object* match_object = new Object(object_name);

    CameraController camera_controller(true);
    //CameraController camera_controller("data/vcan_rotate/", sift_path);
    camera_controller.createViewWindows();

    const Transform t = Common::cameraToWorldSpaceTransform(0.0f, 25.0f);
    while (camera_controller.getNewFrame()) {

        std::vector<SIFTFeature3D> raw_scene_features = camera_controller.getStereoFeatures();
        KinectCamera::CorrelatedImage cframe = camera_controller.getCorrelatedFrame();

        for(unsigned i = 0; i < raw_scene_features.size(); i++){
            raw_scene_features[i].position = t.mat * raw_scene_features[i].position + t.shift;
        }

        std::vector<SIFTFeature3D> scene_features;
        for(unsigned i = 0; i < raw_scene_features.size(); i++){
            if(raw_scene_features[i].position.z > 10.0f && raw_scene_features[i].position.y < 50.0f){
                scene_features.push_back(raw_scene_features[i]);
            }
        }

        std::vector<PointCloudPoint> point_cloud_points;
        for(unsigned i = 0; i < scene_features.size(); i++){
            PointCloudPoint new_point;
            new_point.pos = scene_features[i].position;
            point_cloud_points.push_back(new_point);
        }
        point_cloud->clearPoints();
        point_cloud->addPoints(point_cloud_points);


        Transform object_transform;
        if(match_object->sceneMatch(scene_features, object_transform)){
            match_object->render(object_transform);
        }

        KinectCamera::CorrelatedImage correlated_frame = camera_controller.getCorrelatedFrame();
        IplImage *img = Common::imageFromCorrelatedFrame(correlated_frame);
        for(unsigned i = 0; i < scene_features.size(); i++){
            CvPoint pos = cvPoint(scene_features[i].sift_feature.x,
                                  scene_features[i].sift_feature.y);

            cvCircle(img, pos, 3.0, CV_RGB(0, 0, 255), 1.0);
        }

        if(camera_controller.showImageAndContinue(img) == 'q'){
            cvReleaseImage(&img);
            break;
        }

        cvReleaseImage(&img);
    }

    delete match_object;
}

void MainController::stitchTogether(std::string object0_name, std::string object1_name){
    std::stringstream obj_in_str0, obj_in_str1;
    obj_in_str0 << "data/" << object0_name << "_frames.mf";
    obj_in_str1 << "data/" << object1_name << "_frames.mf";

    SnapshotFrames snapshot_frames0, snapshot_frames1;
    snapshot_frames0.load(obj_in_str0.str());
    snapshot_frames1.load(obj_in_str1.str());

    std::vector<SnapshotFrame> frames0 = snapshot_frames0.getFrames();
    std::vector<SnapshotFrame> frames1 = snapshot_frames1.getFrames();

    for(unsigned i = 0; i < frames0.size(); i++){
        frames0[i].to_camera.print();
    }

    for(unsigned i = 0; i < frames1.size(); i++){
        frames1[i].to_camera.print();
    }

    std::cout << "num frames: " << frames0.size() << " " << frames1.size() << std::endl;

    ReconstructionManager rmanager;
    rmanager.stitchTogether(frames0, frames1);


    std::stringstream obj_out_str;
    obj_out_str << "data/" << object0_name << "_frames.stitched";

    std::ofstream out_file(obj_out_str.str().c_str(), std::ios::binary);
    unsigned num_frames = frames0.size() + frames1.size();
    out_file.write((char*)&num_frames, sizeof(unsigned));

    for(unsigned i = 0; i < frames0.size(); i++){
        frames0[i].writeOutFrame(out_file);
    }

    for(unsigned i = 0; i < frames1.size(); i++){
        frames1[i].writeOutFrame(out_file);
    }

    std::vector<Vector3D> obj_points;
    for(unsigned i = 0; i < frames0.size(); i++){
        for(unsigned j = 0; j < frames0[i].object_pixels.size(); j++){
            obj_points.push_back(frames0[i].object_pixels[j].pos);
        }
    }

    for(unsigned i = 0; i < frames1.size(); i++){
        for(unsigned j = 0; j < frames1[i].object_pixels.size(); j++){
            obj_points.push_back(frames1[i].object_pixels[j].pos);
        }
    }

    obj_points = ICPMatch::filterPoints(obj_points, 0.3f);

    SceneRenderer::instance().initialise();
    PointCloudRenderObject *point_cloud = new PointCloudRenderObject(false, Vector3D(0.0f,1.0f,0.0f));
    SceneRenderer::instance().addObject(point_cloud);

    std::vector<PointCloudPoint> point_cloud_points;
    for(unsigned i = 0; i < obj_points.size(); i++){
        PointCloudPoint new_point;
        new_point.pos = obj_points[i];
        new_point.color = Vector3D(1.0f, 0.0f, 0.0f);
        point_cloud_points.push_back(new_point);
    }

    point_cloud->clearPoints();
    point_cloud->addPoints(point_cloud_points);
}

struct SnapshotFrameComp {
public:
    bool operator()(SnapshotFrame &a, SnapshotFrame &b){
        return a.object_features.size() > b.object_features.size();
    }
};


std::vector<SnapshotFrame> filterFramesByViewDirection(std::vector<SnapshotFrame> in_frames){
    std::sort(in_frames.begin(), in_frames.end(), SnapshotFrameComp());

    const float angle_threshold = 5.0f*M_PI/180.0f;
    std::vector<SnapshotFrame> result;
    for(unsigned i = 0; i < in_frames.size(); i++){
        bool is_clear = true;
        for(unsigned j = 0; j < i; j++){
            in_frames[i].to_camera.normalise();
            in_frames[j].to_camera.normalise();

            float dp = in_frames[i].to_camera.dotProduct(in_frames[j].to_camera);
            if(dp < -1.0f){ dp = -1.0f; }
            if(dp > 1.0f){ dp = 1.0f; }

            float diff = fabs(acosf(dp));
            std::cout << diff << std::endl;
            if(diff < angle_threshold){
                is_clear = false;
                break;
            }
        }

        if(is_clear){
            result.push_back(in_frames[i]);
        }
    }

    return result;
}

void MainController::fitShape(std::string object_name){
    std::stringstream obj_in_str;
    obj_in_str << "data/" << object_name << "_frames.stitched";

    SnapshotFrames snapshot_frames;
    snapshot_frames.load(obj_in_str.str());

    std::vector<SnapshotFrame> frames = snapshot_frames.getFrames();
    std::vector<Vector3D> all_obj_points;
    for(unsigned i = 0; i < frames.size(); i++){
        frames[i].to_camera.print();
        for(unsigned j = 0; j < frames[i].object_features.size(); j++){
            all_obj_points.push_back(frames[i].object_features[j].position);
        }
    }

    all_obj_points = ICPMatch::filterPoints(all_obj_points, 0.2f);
    Vector3D obj_cog = Geometry::centreOfMass(all_obj_points);
    for(unsigned i = 0; i < all_obj_points.size(); i++){
        all_obj_points[i] = all_obj_points[i] - obj_cog;
    }

    SuperQuadricBestFit best_fitter;
    SuperQuadricFit result = best_fitter.calculateShapeHypotheses(all_obj_points);
    std::cout << result.A << " " << result.B << " " << result.C << " " << result.e1 << " " << result.e2 << std::endl;

    for(unsigned i = 0; i < frames.size(); i++){
        frames[i].to_camera = result.transform.mat*frames[i].to_camera;
        for(unsigned j = 0; j < frames[i].object_pixels.size(); j++){
            frames[i].object_pixels[j].pos = 
                result.transform.mat*(frames[i].object_pixels[j].pos - obj_cog) - result.transform.shift;
        }

        for(unsigned j = 0; j < frames[i].object_features.size(); j++){
            frames[i].object_features[j].position = 
                result.transform.mat*(frames[i].object_features[j].position - obj_cog) - result.transform.shift;
        }
    }

    frames = filterFramesByViewDirection(frames);

    std::stringstream obj_out_str;
    obj_out_str << "data/" << object_name << ".final";

    std::ofstream out_file(obj_out_str.str().c_str(), std::ios::binary);
    unsigned num_frames = frames.size();
    out_file.write((char*)&num_frames, sizeof(unsigned));
    std::cout << "writing out num frames: " << num_frames << std::endl;
        
    for(unsigned i = 0; i < frames.size(); i++){
        frames[i].writeOutFrameFeatures(out_file);
    }

    SuperQuadric fitted_sq(result.e1, result.e2, result.A, result.B, result.C);
    fitted_sq.save(out_file);


    SceneRenderer::instance().initialise();
    PointCloudRenderObject *point_cloud = new PointCloudRenderObject(false, Vector3D(0.0f,1.0f,0.0f));
    SceneRenderer::instance().addObject(point_cloud);

    std::vector<PointCloudPoint> point_cloud_points;
    for(unsigned i = 0; i < frames.size(); i++){
        for(unsigned j = 0; j < frames[i].object_features.size(); j++){
            PointCloudPoint new_point;
            new_point.pos = frames[i].object_features[j].position;
            new_point.color = Vector3D(1.0f, 0.0f, 0.0f);
            point_cloud_points.push_back(new_point);
        }
    }

    point_cloud->clearPoints();
    point_cloud->addPoints(point_cloud_points);

    SuperQuadricRenderObject *sq_ro = new SuperQuadricRenderObject(fitted_sq);
    SceneRenderer::instance().addObject(sq_ro);
}

static Vector2D getCorrectedPosition(Vector3D pos, Vector3D blue_pos, Vector3D yellow_pos){
    Vector2D origin((yellow_pos.x + blue_pos.x)/2.0f, (yellow_pos.y + blue_pos.y)/2.0f);

    Vector2D y_axis(yellow_pos.x - origin.x, yellow_pos.y - origin.y);
    y_axis.normalise();
    
    Vector2D x_axis = y_axis;
    x_axis.rotate(-90.0f*M_PI/180.0f);

    Vector2D pos2d(pos.x, pos.y);
    pos2d = pos2d - origin;

    return Vector2D(pos2d.dotProduct(x_axis), pos2d.dotProduct(y_axis));
}

static Transform icpRefine(Object* object, KinectCamera::CorrelatedImage cframe, Transform approx_transform){
    std::vector<Vector3D> src_points = object->getSurfacePoints();
    for(unsigned i = 0; i < src_points.size(); i++){
        src_points[i] = approx_transform.mat*src_points[i] + approx_transform.shift;
    }

    //src_points = ICPMatch::filterPoints(src_points, 0.1f);

    std::vector<Vector3D> dst_points;
    for(unsigned i = 0; i < cframe.depth_pixels.size(); i++){
        if(cframe.depth_pixels[i].have_pos && cframe.depth_pixels[i].pos.z > 17.5f && 
           (approx_transform.shift - cframe.depth_pixels[i].pos).length() < 15.0f){
            dst_points.push_back(cframe.depth_pixels[i].pos);
        }
    }

    std::cout << "filtering points" << std::endl;
    dst_points = ICPMatch::filterPoints(dst_points, 0.2f);
    std::cout << "finished filtering points" << std::endl;

    float err;
    return ICPMatch::match(src_points, dst_points, err);
}

std::pair<float,float> calculateError(Transform object_transform, Vector3D blue_ball_pos, Vector3D yellow_ball_pos,
                                      Vector2D target_pos){
    Vector2D cpos = getCorrectedPosition(object_transform.shift, blue_ball_pos, yellow_ball_pos);
    float pos_error = (cpos - target_pos).length();

    Vector3D main_axis = object_transform.mat*Vector3D(0.0f, 1.0f, 0.0f);
    Vector3D ball_axis = blue_ball_pos - yellow_ball_pos;
    ball_axis.normalise();

    float angle = fabs(acosf(main_axis.dotProduct(ball_axis)));
    float angle_error = 90.0f*M_PI/180.0f - angle;

    return std::pair<float,float>(pos_error, angle_error);
}

void MainController::evalMatch(std::string object_name){
    Vector3D yellow_ball_pos(-0.4f, 22.5f, 20.2f);
    Vector3D blue_ball_pos(-0.5f, 48.0f, 20.7f);

    std::vector<std::string> frame_files;
    for(unsigned side = 0; side < 3; side++){
        for(unsigned i = 0; i < 15; i++){
            std::stringstream str;
            str << "data/" << object_name << "_side" << side << "/output" << i << ".cframe";
            frame_files.push_back(str.str());
        }
    }

    std::vector<Vector2D> frame_positions;
    frame_positions.push_back(Vector2D(-10.0f, 0.0f));
    frame_positions.push_back(Vector2D(0.0f, 0.0f));
    frame_positions.push_back(Vector2D(10.0f, 0.0f));
    frame_positions.push_back(Vector2D(20.0f, 0.0f));
    frame_positions.push_back(Vector2D(20.0f, -10.0f));

    std::cout << "MainController: evalMatch " << object_name << std::endl;

    SceneRenderer::instance().initialise();

    PointCloudRenderObject *point_cloud = new PointCloudRenderObject(false, Vector3D(0.0f,1.0f,0.0f));
    SceneRenderer::instance().addObject(point_cloud);

    SuperQuadricRenderObject *yellow_ball = new SuperQuadricRenderObject(SuperQuadric(1.0f, 1.0f, 2.9f, 2.9f, 2.9f));
    SceneRenderer::instance().addObject(yellow_ball);

    SuperQuadricRenderObject *blue_ball = new SuperQuadricRenderObject(SuperQuadric(1.0f, 1.0f, 2.9f, 2.9f, 2.9f));
    SceneRenderer::instance().addObject(blue_ball);

    Transform ball_transform;
    ball_transform.mat.identity();
    ball_transform.shift = yellow_ball_pos;
    yellow_ball->setTransform(ball_transform);

    ball_transform.shift = blue_ball_pos;
    blue_ball->setTransform(ball_transform);

    Object* match_object = new Object(object_name);

    cvNamedWindow("RGB", 1);
    cvMoveWindow("RGB", 600, 30);

    std::ofstream out_file("truck_detect");

    for(unsigned frame_num = 0; frame_num < frame_files.size(); frame_num++){
        KinectCamera::CorrelatedImage cframe = loadFrame(frame_files[frame_num]);
        IplImage *img = Common::imageFromCorrelatedFrame(cframe);

        std::vector<SIFTFeature3D> raw_scene_features;
        Common::extractFeature(cframe, raw_scene_features);        

        std::vector<SIFTFeature3D> scene_features;
        for(unsigned i = 0; i < raw_scene_features.size(); i++){
            if(raw_scene_features[i].position.z > 10.0f &&
               raw_scene_features[i].position.z < 35.0f &&
               fabs(raw_scene_features[i].position.x) < 40.0f &&
               raw_scene_features[i].position.y > 20.0f && 
               raw_scene_features[i].position.y < 50.0f){
                scene_features.push_back(raw_scene_features[i]);
            }
        }

        std::vector<PointCloudPoint> point_cloud_points;
        /*
        for(unsigned i = 0; i < cframe.depth_pixels.size(); i++){
            if(cframe.depth_pixels[i].have_pos &&
               cframe.depth_pixels[i].pos.z > 10.0f && 
               cframe.depth_pixels[i].pos.z < 35.0f &&
               fabs(cframe.depth_pixels[i].pos.x) < 40.0f &&
               cframe.depth_pixels[i].pos.y > 20.0f && 
               cframe.depth_pixels[i].pos.y < 60.0f){
                PointCloudPoint new_point;
                new_point.pos = cframe.depth_pixels[i].pos;
                new_point.color = cframe.depth_pixels[i].color;
                new_point.color.scale(1.0f/255.0f);
                point_cloud_points.push_back(new_point);
            }
        }
        */

        for(unsigned i = 0; i < scene_features.size(); i++){
            PointCloudPoint new_point;
            new_point.pos = scene_features[i].position;
            point_cloud_points.push_back(new_point);
        }

        point_cloud->clearPoints();
        point_cloud->addPoints(point_cloud_points);

        Transform object_transform;
        if(match_object->sceneMatch(scene_features, object_transform)){
            std::pair<float,float> sift_error = 
                calculateError(object_transform, blue_ball_pos, yellow_ball_pos, frame_positions[(frame_num%15)/3]);

            std::cout << "doing icp refine" << std::endl;
            Transform icpt = icpRefine(match_object, cframe, object_transform);
            std::cout << "finished icp refine" << std::endl;
            object_transform.mat = icpt.mat*object_transform.mat;
            object_transform.shift = icpt.mat*object_transform.shift + icpt.shift;

            std::pair<float,float> icp_error = 
                calculateError(object_transform, blue_ball_pos, yellow_ball_pos, frame_positions[(frame_num%15)/3]);

            match_object->render(object_transform);

            out_file << sift_error.first << "," << sift_error.second << "," << icp_error.first << "," << icp_error.second << std::endl;
            std::cout << "sift pos/angle error: " << sift_error.first << " " << sift_error.second << std::endl;
            std::cout << "icp pos/angle error: " << icp_error.first << " " << icp_error.second << std::endl;
        }       

        cvShowImage("RGB", img);
        cvWaitKey(5);
        cvReleaseImage(&img);
    }

    delete match_object;
}

void MainController::finalisePointCloud(std::string object_name){
    std::stringstream obj_in_str;
    obj_in_str << "data/" << object_name << "_frames.stitched";

    SnapshotFrames snapshot_frames;
    snapshot_frames.load(obj_in_str.str());

    std::vector<SnapshotFrame> frames = snapshot_frames.getFrames();
    std::vector<Vector3D> all_obj_points;
    for(unsigned i = 0; i < frames.size(); i++){
        frames[i].to_camera.print();
        for(unsigned j = 0; j < frames[i].object_pixels.size(); j++){
            all_obj_points.push_back(frames[i].object_pixels[j].pos);
        }
    }

    std::vector<std::vector<Vector3D> > buckets;
    std::set<unsigned> filled_buckets;
    unsigned xsize, ysize, zsize;

    const float grid_size = 0.25f;
    Vector3D corner;
    ICPMatch::buildBuckets(all_obj_points, grid_size, buckets, filled_buckets, corner, xsize, ysize, zsize);

    std::list<unsigned> bucket_index_stack;
    std::vector<std::set<unsigned> > bucket_labels;

    while(filled_buckets.size() > 0){
        bucket_index_stack.clear();
        std::set<unsigned>::iterator it;

        it = filled_buckets.begin();
        bucket_index_stack.push_front(*it);
        filled_buckets.erase(it);

        std::set<unsigned> new_label_set;
        while(bucket_index_stack.size() > 0){
            unsigned cur = bucket_index_stack.back();
            bucket_index_stack.pop_back();
            new_label_set.insert(cur);

            unsigned cur_x = cur % xsize;
            unsigned cur_y = (cur / xsize) % ysize;
            unsigned cur_z = (cur / xsize) / ysize;

            unsigned n0 = (cur_x+1) + cur_y*xsize + cur_z*xsize*ysize;
            unsigned n1 = (cur_x-1) + cur_y*xsize + cur_z*xsize*ysize;
            unsigned n2 = cur_x + (cur_y+1)*xsize + cur_z*xsize*ysize;
            unsigned n3 = cur_x + (cur_y-1)*xsize + cur_z*xsize*ysize;
            unsigned n4 = cur_x + cur_y*xsize + (cur_z+1)*xsize*ysize;
            unsigned n5 = cur_x + cur_y*xsize + (cur_z-1)*xsize*ysize;

            if(filled_buckets.find(n0) != filled_buckets.end()){ filled_buckets.erase(n0); bucket_index_stack.push_front(n0); }
            if(filled_buckets.find(n1) != filled_buckets.end()){ filled_buckets.erase(n1); bucket_index_stack.push_front(n1); }
            if(filled_buckets.find(n2) != filled_buckets.end()){ filled_buckets.erase(n2); bucket_index_stack.push_front(n2); }
            if(filled_buckets.find(n3) != filled_buckets.end()){ filled_buckets.erase(n3); bucket_index_stack.push_front(n3); }
            if(filled_buckets.find(n4) != filled_buckets.end()){ filled_buckets.erase(n4); bucket_index_stack.push_front(n4); }
            if(filled_buckets.find(n5) != filled_buckets.end()){ filled_buckets.erase(n5); bucket_index_stack.push_front(n5); }
        }

        bucket_labels.push_back(new_label_set);
    }

    std::set<unsigned> largest_bucket;
    for(unsigned i = 0; i < bucket_labels.size(); i++){
        if(bucket_labels[i].size() > largest_bucket.size()){
            largest_bucket = bucket_labels[i];
        }
    }

    SceneRenderer::instance().initialise();

    PointCloudRenderObject *point_cloud = new PointCloudRenderObject(false, Vector3D(0.0f,1.0f,0.0f));
    SceneRenderer::instance().addObject(point_cloud);

    std::vector<PointCloudPoint> point_cloud_points;

    std::vector<Vector3D> bucket_points;
    std::set<unsigned>::iterator it;
    for(it = largest_bucket.begin(); it != largest_bucket.end(); ++it){
        bucket_points.push_back(Geometry::centreOfMass(buckets[*it]));
    }

    Transform centralised_transform;
    CentralisePoints cp;
    cp.calculateTransform(bucket_points, centralised_transform);

    

    std::stringstream obj_out_str;
    obj_out_str << "data/" << object_name << ".final";

    std::ofstream out_file(obj_out_str.str().c_str(), std::ios::binary);
    unsigned num_frames = frames.size();
    out_file.write((char*)&num_frames, sizeof(unsigned));
    std::cout << "writing out num frames: " << num_frames << std::endl;

    for(unsigned i = 0; i < frames.size(); i++){
        std::vector<SIFTFeature3D> filtered_features;
        for(unsigned j = 0; j < frames[i].object_features.size(); j++){
            

            unsigned xindex = (frames[i].object_features[j].position.x - corner.x)/grid_size;
            unsigned yindex = (frames[i].object_features[j].position.y - corner.y)/grid_size;
            unsigned zindex = (frames[i].object_features[j].position.z - corner.z)/grid_size;
            unsigned index = xindex + yindex*xsize + zindex*xsize*ysize;

            if(largest_bucket.find(index) != largest_bucket.end()){
                frames[i].object_features[j].position = 
                    centralised_transform.mat*frames[i].object_features[j].position + centralised_transform.shift;

                PointCloudPoint new_point;
                new_point.pos = frames[i].object_features[j].position;
                new_point.color = Vector3D(1.0f, 0.0f, 0.0f);
                point_cloud_points.push_back(new_point);

                filtered_features.push_back(frames[i].object_features[j]);
            }
        }

        frames[i].object_features = filtered_features;
        frames[i].writeOutFrameFeatures(out_file);
    }

    unsigned num_surface_points = bucket_points.size();
    out_file.write((char*)&num_surface_points, sizeof(unsigned));
    for(unsigned i = 0; i < bucket_points.size(); i++){
        bucket_points[i] = centralised_transform.mat*bucket_points[i] + centralised_transform.shift;
        out_file.write((char*)&bucket_points[i], sizeof(Vector3D));

        PointCloudPoint new_point;
        new_point.pos = bucket_points[i];
        new_point.color = Vector3D(0.0f, 1.0f, 0.0f);
        point_cloud_points.push_back(new_point);
    }
    
    point_cloud->clearPoints();
    point_cloud->addPoints(point_cloud_points);
}

void MainController::mouseHandler(int event, int x, int y, int left_or_right){

}
