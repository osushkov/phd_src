
#include "CorrelatedFrameLearner.h"
#include "../Settings.h"
#include "../Features/SIFTFeature3D.h"
#include "../Features/FeatureMemory/FeatureMemory.h"
//#include "../Evaluation/MemoryEvaluator.h"
#include "../Reconstruction/ReconstructionManager.h"

#include <vector>
#include <fstream>

CorrelatedFrameLearner::CorrelatedFrameLearner() : parser(NULL) {

}

CorrelatedFrameLearner::~CorrelatedFrameLearner(){
    if(parser != NULL){
        delete parser;
    }
}

void CorrelatedFrameLearner::loadCorrelatedFrames(std::string path){
    if(parser != NULL){
        delete parser;
    }
    parser = new CorrelatedFrameParser (path);
    std::vector<CorrelatedFrame> all_frames = parser->read();

    for(unsigned i = 0; i < all_frames.size(); i++){
        std::map<std::string, std::vector<CorrelatedFrame> >::iterator it =
            object_correlated_frames.find(all_frames[i].object_type);

        if(it != object_correlated_frames.end()){
            it->second.push_back(all_frames[i]);
        }
        else{
            std::vector<CorrelatedFrame> new_entry;
            new_entry.push_back(all_frames[i]);
            object_correlated_frames.insert(make_pair(all_frames[i].object_type, new_entry));
        }
    }
}

// Correlate all of the frames of the given object with random frames of other
// objects.
// 1. get all of the CorrelatedFrames of the target object.
// 2. Go through all other objects and pick out some random frames from each object.
// 3. Try to find matching frames between every frame in the target object frame set
//    and the other object frames.
// 4. For every matching frame pair, rule out certain features if they co-occur.
// 5. Write back the filtered target object frames to disk.
void CorrelatedFrameLearner::processCorrelatedFrames(std::string object_name){
    std::map<std::string, std::vector<CorrelatedFrame> >::iterator it =
        object_correlated_frames.find(object_name);

    if(it == object_correlated_frames.end()){
        std::cout << "Object " << object_name << " not found" << std::endl;
        return;
    }

    std::vector<CorrelatedFrame> object_features = it->second;
    std::vector<CorrelatedFrame> other_objects;

    for(it = object_correlated_frames.begin(); it != object_correlated_frames.end(); ++it){
        if(it->first == object_name){
            continue;
        }

        std::vector<CorrelatedFrame> current_features = it->second;
        for(unsigned i = 0; i < current_features.size(); i++){
            other_objects.push_back(current_features[i]);
        }
    }

    std::cout << "Total " << object_name << " features: " << object_features.size() << std::endl;
    std::cout << "Total other features: " << other_objects.size() << std::endl;

    doCorrelation(object_features, other_objects, object_name);
}

void CorrelatedFrameLearner::writeOutCorrelatedFrames(std::string filename){

}

void CorrelatedFrameLearner::doCorrelation(std::vector<CorrelatedFrame> &object_frames,
                                           std::vector<CorrelatedFrame> &reference_frames,
                                           std::string object_name){

    ReconstructionManager reconstruction_manager("sprite1");
    FeatureMemory feature_memory;
    feature_memory.load("data/feature_memory.dat");

    /*MemoryEvaluator memory_evaluator("data/verification/cat3_clean_left.png",
                                     "data/verification/cat3_clean_right.png",
                                     "data/verification/cat3_clean_left_mask.png",
                                     "data/verification/cat3_clean_right_mask.png");
    memory_evaluator.evaluateBase();*/

    //const unsigned num_random_lookups = 500; // TODO: make this a setting or something

    std::cout << "num object frames: " << object_frames.size() << std::endl;
    std::cout << "num reference frames: " << reference_frames.size() << std::endl;

    std::fstream tmp_file;
    tmp_file.open("tmp_file.dat", std::fstream::in);
    for(unsigned i = 0; i < object_frames.size(); i++){
        parser->loadCorrelatedFrame(object_frames[i]);

        CorrelatedFrame old_frame = object_frames[i];
        num_corrections = 0;

        for(unsigned j = 0; j < reference_frames.size(); j++){
            unsigned reference_index = j;//rand()%reference_frames.size();
            if(!reference_frames[reference_index].fully_loaded){
                parser->loadCorrelatedFrame(reference_frames[reference_index]);
            }
            //if(isSignificantCorrelation(object_frames[i], reference_frames[reference_index])){
                //std::cout << "Significant correlation : " << i << " " << reference_index
                //          << std::endl;
                doFrameCorrelation(object_frames[i], reference_frames[reference_index]);
            //}
        }

        std::vector<SIFTFeature3D> object_features;
        for(unsigned j = 0; j < object_frames[i].feature_types.size(); j++){
            if(object_frames[i].feature_types[j] == OBJECT_FEATURE){
                object_features.push_back(object_frames[i].features[j]);

                feature_memory.insertFeature(OBJECT_FEATURE,
                                             object_frames[i].features[j].sift_feature,
                                             object_name);

            }
        }
/*
        std::vector<float> arm_joints;
        unsigned num_joints;
        tmp_file >> num_joints;
        assert(num_joints == 6);
        for(unsigned j = 0; j < num_joints; j++){
            float joint;
            tmp_file >> joint;
            arm_joints.push_back(joint);
        }

        //reconstruction_manager.submitFeatures(object_features, arm_joints);
*/
        //reconstruction_manager.submitFeatures(object_features);
/*
        if (num_corrections > 0 || true) {
            std::cout << "num corrections: " << num_corrections << std::endl;

            cvNamedWindow("New", 1);
            renderCorrelatedFrame(object_frames[i], "New");

            std::cout << "Proceed?" << std::endl;
            cvWaitKey();

            for (unsigned i = 0; i < 2; i++) {
                cvDestroyWindow("New");
                cvWaitKey(5);
            }

            num_corrections = 0;
        }
*/

    }

    //std::vector<Vector3D> object_points = reconstruction_manager.reconstruct();

    //feature_memory.rebuild();
    //memory_evaluator.evaluate(feature_memory, object_name);

    feature_memory.save("data/feature_memory.dat");
}

bool CorrelatedFrameLearner::isSignificantCorrelation(CorrelatedFrame &object_frame,
                                                      CorrelatedFrame &reference_frame){
    assert(object_frame.fully_loaded && reference_frame.fully_loaded);

    const float min_same_feature_dist = 500.0f; // TODO make this a setting or something
    const unsigned min_required_same_features = 10;

    unsigned num_same_features = 0;

    for (unsigned i = 0; i < object_frame.features.size(); i++) {
        for (unsigned j = 0; j < reference_frame.features.size(); j++) {
            float feature_dist = Common::featureDist(object_frame.features[i],
                                                     reference_frame.features[j]);
            if (feature_dist < min_same_feature_dist) {
                num_same_features++;
            }
        }
    }

	return num_same_features >= min_required_same_features;
}

void CorrelatedFrameLearner::doFrameCorrelation(CorrelatedFrame &object_frame,
                                                CorrelatedFrame &reference_frame){

    const float min_same_feature_dist = 400.0f;

    // very simple correlation for now.
    for(unsigned i = 0; i < object_frame.features.size(); i++){
        if(object_frame.feature_types[i] == ARM_FEATURE){
            continue;
        }

        for(unsigned j = 0; j < reference_frame.features.size(); j++){
            float feature_dist = Common::featureDist(object_frame.features[i],
                                                     reference_frame.features[j]);
            if (feature_dist < min_same_feature_dist) {
                object_frame.feature_types[i] = ARM_FEATURE;
                num_corrections++;
                break;
            }
        }
    }
}

void CorrelatedFrameLearner::renderCorrelatedFrame(CorrelatedFrame &frame,
                                                   std::string window_name){

    std::string filename =
            Settings::instance().getStringValue("general", "correlated_features_path") +
            frame.rgb_frame_filename;


    IplImage *img = cvLoadImage(filename.c_str());
    for (unsigned i = 0; i < frame.features.size(); i++) {
        CvScalar col;

        if(frame.feature_types[i] == ARM_FEATURE){
            col = CV_RGB(0, 0, 255);
        }
        else if(frame.feature_types[i] == OBJECT_FEATURE){
            col = CV_RGB(0, 255, 0);
        }
        else{
            col = CV_RGB(255, 255, 255);
        }

        cvCircle(img,
                 cvPoint(frame.features[i].sift_feature.x, frame.features[i].sift_feature.y),
                 1, col, 2.0);
    }

    cvShowImage(window_name.c_str(), img);
}

