
#include "SnapshotFrames.h"
#include "../Util/Common.h"
#include <fstream>

SnapshotFrames::SnapshotFrames(){

}

SnapshotFrames::~SnapshotFrames(){

}

void SnapshotFrames::addFrame(const SnapshotFrame &frame){
    snapshot_frames.push_back(frame);
}

std::vector<SnapshotFrame> SnapshotFrames::getFrames(void){
    return snapshot_frames;
}

void SnapshotFrames::save(std::string filename){
    std::ofstream out_file(filename.c_str(), std::ios::binary);
    
    unsigned num_frames = snapshot_frames.size();
    out_file.write((char*)&num_frames, sizeof(unsigned));

    for(unsigned i = 0; i < snapshot_frames.size(); i++){
        snapshot_frames[i].writeOutFrame(out_file);
    }
}

bool SnapshotFrames::load(std::string filename){
    std::ifstream in_file(filename.c_str(), std::ios::binary);

    if(!in_file.good() || in_file.bad() || in_file.eof()){
        return false;
    }

    snapshot_frames.clear();

    unsigned num_frames = 0;
    in_file.read((char*)&num_frames, sizeof(unsigned));

    for(unsigned i = 0; i < num_frames; i++){
        SnapshotFrame new_frame = SnapshotFrame::readFrame(in_file);
        snapshot_frames.push_back(new_frame);
    }

    return true;
}

void SnapshotFrame::writeOutFrame(std::ostream &out_file){
    out_file.write((char*)&(to_camera), sizeof(Vector3D));

    unsigned num_arm_features = arm_features.size();
    out_file.write((char*)&num_arm_features, sizeof(unsigned));
    for(unsigned i = 0; i < arm_features.size(); i++){
        Common::writeFeature(out_file, arm_features[i]);
    }

    unsigned num_object_features = object_features.size();
    out_file.write((char*)&num_object_features, sizeof(unsigned));
    for(unsigned i = 0; i < object_features.size(); i++){
        Common::writeFeature(out_file, object_features[i]);
    }

    unsigned num_arm_pixels = arm_pixels.size();
    out_file.write((char*)&num_arm_pixels, sizeof(unsigned));
    for(unsigned i = 0; i < arm_pixels.size(); i++){
        out_file.write((char*)&(arm_pixels[i].color), sizeof(Vector3D));
        out_file.write((char*)&(arm_pixels[i].pos), sizeof(Vector3D));
    }

    unsigned num_object_pixels = object_pixels.size();
    out_file.write((char*)&num_object_pixels, sizeof(unsigned));
    for(unsigned i = 0; i < object_pixels.size(); i++){
        out_file.write((char*)&(object_pixels[i].color), sizeof(Vector3D));
        out_file.write((char*)&(object_pixels[i].pos), sizeof(Vector3D));
    }
}

void SnapshotFrame::writeOutFrameFeatures(std::ostream &out_file){
    out_file.write((char*)&(to_camera), sizeof(Vector3D));

    unsigned num_object_features = object_features.size();
    out_file.write((char*)&num_object_features, sizeof(unsigned));
    for(unsigned i = 0; i < object_features.size(); i++){
        Common::writeFeature(out_file, object_features[i]);
    }
}

SnapshotFrame SnapshotFrame::readFrame(std::istream &in_file){
    SnapshotFrame result;

    in_file.read((char*)&(result.to_camera), sizeof(Vector3D));

    unsigned num_arm_features = 0;
    in_file.read((char*)&num_arm_features, sizeof(unsigned));
    for(unsigned i = 0; i < num_arm_features; i++){
        SIFTFeature3D new_arm_feature;
        Common::readFeature(in_file, new_arm_feature);
        result.arm_features.push_back(new_arm_feature);
    }

    unsigned num_object_features = 0;
    in_file.read((char*)&num_object_features, sizeof(unsigned));
    for(unsigned i = 0; i < num_object_features; i++){
        SIFTFeature3D new_object_feature;
        Common::readFeature(in_file, new_object_feature);
        result.object_features.push_back(new_object_feature);
    }

    unsigned num_arm_pixels = 0;
    in_file.read((char*)&num_arm_pixels, sizeof(unsigned));
    for(unsigned i = 0; i < num_arm_pixels; i++){
        KinectCamera::DepthPixel new_arm_pixel;
        in_file.read((char*)&(new_arm_pixel.color), sizeof(Vector3D));
        in_file.read((char*)&(new_arm_pixel.pos), sizeof(Vector3D));
        result.arm_pixels.push_back(new_arm_pixel);
    }

    unsigned num_object_pixels = 0;
    in_file.read((char*)&num_object_pixels, sizeof(unsigned));
    for(unsigned i = 0; i < num_object_pixels; i++){
        KinectCamera::DepthPixel new_object_pixel;
        in_file.read((char*)&(new_object_pixel.color), sizeof(Vector3D));
        in_file.read((char*)&(new_object_pixel.pos), sizeof(Vector3D));
        result.object_pixels.push_back(new_object_pixel);
    }

    return result;
}

SnapshotFrame SnapshotFrame::readFrameFeatures(std::istream &in_file){
    SnapshotFrame result;

    in_file.read((char*)&(result.to_camera), sizeof(Vector3D));

    unsigned num_object_features = 0;
    in_file.read((char*)&num_object_features, sizeof(unsigned));
    for(unsigned i = 0; i < num_object_features; i++){
        SIFTFeature3D new_object_feature;
        Common::readFeature(in_file, new_object_feature);
        result.object_features.push_back(new_object_feature);
    }

    return result;
}

