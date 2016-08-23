
#include "CorrelatedFrameParser.h"
#include "../Settings.h"

#include <iostream>
#include <sstream>

CorrelatedFrameParser::CorrelatedFrameParser(std::string path) : path(path) {
    assert(path.size() > 0);

    if (path[path.size() - 1] != '/') {
        path = path + "/";
    }

    std::string root_filename = path + "root.dat";

    // Try to open the file, if it does not exist, start from output number 0.
    // If it does exist, read the present entries to determine which output
    // number to start from.
    root_file.open(root_filename.c_str(), std::fstream::in);
    if(!root_file.is_open()){
        cur_output_num = 0;
    }
    else{
        while(!root_file.eof()){
            std::string tmp;
            root_file >> cur_output_num;
            root_file >> tmp;
            root_file >> tmp;
            root_file >> tmp;
            root_file >> tmp;
        }
        cur_output_num++;
    }
    root_file.close();

    root_file.open(root_filename.c_str(), std::fstream::in|std::fstream::out|std::fstream::app);
    std::cout << "Current output number: " << cur_output_num << std::endl;
}


CorrelatedFrameParser::~CorrelatedFrameParser(){

}


void CorrelatedFrameParser::write(IplImage *rgb_frame,
                                  const std::vector<SIFTFeature3D> &features,
                                  const std::vector<FeatureObjectType> &feature_types,
                                  std::string object_type){
    std::cout << "Saving correlated frame num: " << cur_output_num << std::endl;

    assert(rgb_frame != NULL);

    std::stringstream rgb_frame_filename;
    rgb_frame_filename << "rgb_" << cur_output_num << ".png";

    cvSaveImage((path+rgb_frame_filename.str()).c_str(), rgb_frame);

    std::stringstream features_filename;
    features_filename << "features_"  << cur_output_num << ".dat";

    std::ofstream features_file((path+features_filename.str()).c_str(), std::ios::binary);
    writeOutFeatures(features_file, features, feature_types);

    root_file << cur_output_num << " "
              << rgb_frame_filename.str() << " "
              << features_filename.str() << " "
              << object_type << std::endl;

    cur_output_num++;
}


std::vector<CorrelatedFrame> CorrelatedFrameParser::read(void){
    std::string root_filename = path + "root.dat";

    std::vector<CorrelatedFrame> result;
    std::fstream in_file(root_filename.c_str(), std::fstream::in);

    while(!in_file.eof()){
        unsigned output_num;
        in_file >> output_num;

        CorrelatedFrame new_frame;
        in_file >> new_frame.rgb_frame_filename;
        in_file >> new_frame.features_filename;
        in_file >> new_frame.object_type;
        new_frame.fully_loaded = false;

        result.push_back(new_frame);
    }

    return result;
}


void CorrelatedFrameParser::loadCorrelatedFrame(CorrelatedFrame &frame){
    std::string filename =
        Settings::instance().getStringValue("general", "correlated_features_path") +
        frame.features_filename;

    std::ifstream in_file(filename.c_str());
    assert(in_file.is_open());
    readInFeatures(in_file, frame.features, frame.feature_types);
    frame.fully_loaded = true;
}


void CorrelatedFrameParser::writeOutFeatures(std::ofstream &out_file,
                                             const std::vector<SIFTFeature3D> &features,
                                             const std::vector<FeatureObjectType> &feature_types){

    assert(features.size() == feature_types.size());

    unsigned num_features = features.size();
    out_file.write((char*)&num_features, sizeof(unsigned));

    for(unsigned i = 0; i < features.size(); i++){

        write_feature(out_file, features[i].sift_feature);

        out_file.write((char*)&(features[i].position.x), sizeof(float));
        out_file.write((char*)&(features[i].position.y), sizeof(float));
        out_file.write((char*)&(features[i].position.z), sizeof(float));

        //TODO: write out the feature types too.
        unsigned feature_token = 0;
        switch(feature_types[i]){
        case ARM_FEATURE:
            feature_token = 1;
            break;
        case BACKGROUND_FEATURE:
            feature_token = 2;
            break;
        case OBJECT_FEATURE:
            feature_token = 3;
            break;
        default:
            feature_token = 0;
            break;
        }
        out_file.write((char*)&feature_token, sizeof(unsigned));
    }
}


void CorrelatedFrameParser::readInFeatures(std::ifstream &in_file,
                                           std::vector<SIFTFeature3D> &features,
                                           std::vector<FeatureObjectType> &feature_types){
    features.clear();
    feature_types.clear();

    unsigned num_features = 0;
    in_file.read((char*)&num_features, sizeof(unsigned));

    for(unsigned i = 0; i < num_features; i++){
        SIFTFeature3D new_feature;

        read_feature(in_file, new_feature.sift_feature);

        in_file.read((char*)&(new_feature.position.x), sizeof(float));
        in_file.read((char*)&(new_feature.position.y), sizeof(float));
        in_file.read((char*)&(new_feature.position.z), sizeof(float));

        FeatureObjectType type = BACKGROUND_FEATURE;
        unsigned feature_token = 0;
        in_file.read((char*)&feature_token, sizeof(unsigned));

        switch (feature_token) {
        case 1:
            type = ARM_FEATURE;
            break;
        case 2:
            type = BACKGROUND_FEATURE;
            break;
        case 3:
            type = OBJECT_FEATURE;
            break;
        }

        features.push_back(new_feature);
        feature_types.push_back(type);
    }
}


std::string CorrelatedFrameParser::pathToRootFile(std::string path){
    return "";
}

