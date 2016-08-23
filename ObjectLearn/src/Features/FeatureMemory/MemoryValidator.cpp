/*
 * MemoryValidator.cpp
 *
 *  Created on: 02/09/2009
 *      Author: osushkov
 */

#include "MemoryValidator.h"
#include <fstream>
#include <sstream>

#include "ObjectSnapshotDBMono.h"
#include "FeatureMemory.h"
#include "../../Util/Timer.h"

MemoryValidator::MemoryValidator() : images_per_object(12) {

}

MemoryValidator::~MemoryValidator(){

}

void MemoryValidator::load(std::string path){
    std::string list_filename = path + "list.txt";
    std::ifstream list_file(list_filename.c_str());

    list_file >> num_objects;

    for(unsigned i = 0; i < num_objects; i++){
        std::string obj_name;
        list_file >> obj_name;

        for(unsigned num = 0; num < 4; num++){
            std::stringstream tmp_stream;

            tmp_stream << path << obj_name << "_0lights_" << num << ".png";
            IplImage *img = cvLoadImage(tmp_stream.str().c_str());
            std::cout << tmp_stream.str() << std::endl;
            assert(img != NULL);
            images.push_back(img);
            tmp_stream.str("");

            tmp_stream << path << obj_name << "_1light_" << num << ".png";
            img = cvLoadImage(tmp_stream.str().c_str());
            std::cout << tmp_stream.str() << std::endl;
            assert(img != NULL);
            images.push_back(img);
            tmp_stream.str("");

            tmp_stream << path << obj_name << "_2lights_" << num << ".png";
            img = cvLoadImage(tmp_stream.str().c_str());
            std::cout << tmp_stream.str() << std::endl;
            assert(img != NULL);
            images.push_back(img);
            tmp_stream.str("");

            tmp_stream << path << obj_name << "_0lights_" << num << "_mask.png";
            img = cvLoadImage(tmp_stream.str().c_str());
            std::cout << tmp_stream.str() << std::endl;
            assert(img != NULL);
            masks.push_back(img);
            tmp_stream.str("");

            tmp_stream << path << obj_name << "_1light_" << num << "_mask.png";
            img = cvLoadImage(tmp_stream.str().c_str());
            std::cout << tmp_stream.str() << std::endl;
            assert(img != NULL);
            masks.push_back(img);
            tmp_stream.str("");

            tmp_stream << path << obj_name << "_2lights_" << num << "_mask.png";
            img = cvLoadImage(tmp_stream.str().c_str());
            std::cout << tmp_stream.str() << std::endl;
            assert(img != NULL);
            masks.push_back(img);
            tmp_stream.str("");
        }
    }

    /*
    std::string list_filename2 = "RandomImages/file_list.txt";
    std::ifstream list_file2(list_filename2.c_str());

    list_file2 >> num_objects;

    for(unsigned i = 0; i < num_objects; i++){
        std::string obj_name;
        list_file2 >> obj_name;

        obj_name = "RandomImages/" + obj_name;
        std::cout << obj_name << std::endl;

        IplImage *img = cvLoadImage(obj_name.c_str());
        assert(img != NULL);
        masks.push_back(img);
        images.push_back(img);
    }
    */

    for(unsigned i = 0; i < images.size(); i++){
        feature *image_features;
        unsigned num_features = SIFT::sift_features(images[i], &image_features);

        std::vector<feature> object_features;
        for(unsigned j = 0; j < num_features; j++){
            object_features.push_back(image_features[j]);
        }
        features.push_back(object_features);
        free(image_features);
    }
}

void MemoryValidator::evaluate(void){
    cdetected = new std::vector<float>[images_per_object];
    fdetected = new std::vector<float>[images_per_object];

    params = std::vector<float>(5);
    params[0] = 8.0f;
    params[1] = 0.6f;
    params[2] = 0.8f;
    params[3] = 0.15f;
    params[4] = 0.25f;

    //for (float p = 0.05f; p < 0.5f; p += 0.05f) {


    for (unsigned object_num = 0; object_num < num_objects; object_num++) {
        for(unsigned i = 0; i < 6; i++){
            std::cout << object_num << " : " << i << std::endl;
            unsigned mask = ~(1 << (i*2)) & 4095;

            //while((mask = 1 + rand()%4094) && numOnes(mask) != 11);
            std::cout << "m: " << mask << std::endl;
            evaluateSet(object_num, mask);
        }
    }

    for (unsigned i = 0; i < images_per_object; i++) {
        float cavrg = Common::average(cdetected[i]);
        float cci = 1.96f*Common::standardDeviation(cdetected[i])/sqrtf(cdetected[i].size());

        float favrg = Common::average(fdetected[i]);
        float fci = 1.96f*Common::standardDeviation(fdetected[i])/sqrtf(fdetected[i].size());
        std::cout << i << " : (" << cavrg << "," << cci << ") (" << favrg << "," << fci << ")" << std::endl;

    }

    return;

    for(unsigned i = 0; i < 1000; i++){
        unsigned num_learn_snapshots = rand()%images.size();
        if(num_learn_snapshots == 0){
            num_learn_snapshots = 1;
        }
/*
        FeatureMemory feature_memory;
        //feature_memory.load("data/feature_memory_base.dat");

        for(unsigned j = 0; j < num_learn_snapshots; j++){
            unsigned index = rand() % images.size();
            for(unsigned k = 0; k < features[index].size(); k++){
                if(isObjectFeature(features[index].at(k), masks[index])){
                    if(rand()%4 == 0){
                        feature_memory.insertFeature(BACKGROUND_FEATURE, features[index].at(k), "");
                    }
                    else{
                        feature_memory.insertFeature(OBJECT_FEATURE, features[index].at(k), "");
                    }
                }
            }
        }
        feature_memory.rebuild();

        const unsigned iters = 5;
        unsigned classified = 0;
        Util::Timer timer;
        timer.start();
        for(unsigned j = 0; j < iters; j++){
            unsigned index = rand()%images.size();
            for(unsigned k = 0; k < features[index].size(); k++){
                std::string tmp;
                feature_memory.classifyFeature(features[index].at(k), APPROX_NN, tmp);
                classified++;
            }
        }
        timer.stop();

        std::cout << feature_memory.getNumFeatures() << ","
                  << timer.getNumElapsedSeconds()*1000.0f/classified << std::endl;
*/


        ObjectSnapshotDBMono *snapshot_db = new ObjectSnapshotDBMono();
        snapshot_db->setParams(params);

        for(unsigned j = 0; j < num_learn_snapshots; j++){
            unsigned index = rand()%images.size();
            std::stringstream tmp_stream;
            tmp_stream << "obj" << index/12;

            std::vector<feature> object_features;
            for(unsigned k = 0; k < features[index].size(); k++){
                if(isObjectFeature(features[index].at(k), masks[index])){
                    object_features.push_back(features[index].at(k));
                }
            }
            snapshot_db->addSnapshot(tmp_stream.str(), object_features);
        }

        const unsigned iters = 5;
        unsigned classified = 0;
        Util::Timer timer;
        timer.start();
        for(unsigned j = 0; j < iters; j++){
                unsigned index = rand()%images.size();
                std::stringstream tmp_stream;
                tmp_stream << "obj" << index/12;

                snapshot_db->matchScene(features[index], tmp_stream.str());
                classified += features[index].size();
        }
        timer.stop();

        std::cout << snapshot_db->getNumFeatures() << ","
                  << timer.getNumElapsedSeconds()*1000.0f/classified << std::endl;

        delete snapshot_db;

    }

}

bool MemoryValidator::isObjectFeature(feature &f, IplImage *mask){
    CvScalar s = cvGet2D(mask, f.y, f.x);
    return s.val[0] > 30;
}

float MemoryValidator::evaluateSet(unsigned object_num, unsigned learning_set){
    unsigned num_ones = numOnes(learning_set);

    std::vector<unsigned> learn_set, eval_set;
    unsigned total_detected_correct = 0, total_detected_false = 0, total_object_features = 0;

    int third = 0;
    for(unsigned i = 0; i < images_per_object; i++){
        if((learning_set & (1<<i)) != 0){
            learn_set.push_back(object_num*images_per_object + i);
            third = i/4;
        }
        else{
            int offset = 0; //rand()%(num_objects-1);
            eval_set.push_back(((object_num+offset)%num_objects)*images_per_object + i);
        }
    }

/*
    ObjectSnapshotDBMono *snapshot_db = new ObjectSnapshotDBMono();
    snapshot_db->setParams(params);
    for(unsigned i = 0; i < learn_set.size(); i++){
        std::vector<feature> object_features;
        for(unsigned j = 0; j < features[learn_set[i]].size(); j++){
            if(isObjectFeature(features[learn_set[i]].at(j), masks[learn_set[i]])){
                object_features.push_back(features[learn_set[i]].at(j));
            }
        }
        snapshot_db->addSnapshot("obj", object_features);
    }

    for(unsigned i = 0; i < eval_set.size(); i++){
        //if((eval_set[i]-object_num*images_per_object)/4 == third){ continue; }

        std::set<unsigned> detected_features = snapshot_db->matchScene(features[eval_set[i]]);

        for(unsigned j = 0; j < features[eval_set[i]].size(); j++){
            if(isObjectFeature(features[eval_set[i]].at(j), masks[eval_set[i]])){
                total_object_features++;
                if(detected_features.find(j) != detected_features.end()){
                    total_detected_correct++;
                }
            }
        }

        std::set<unsigned>::iterator it;
        for(it = detected_features.begin(); it != detected_features.end(); ++it){
            if(!isObjectFeature(features[eval_set[i]].at(*it), masks[eval_set[i]]) &&
               edgeDist(features[eval_set[i]].at(*it).x,
                        features[eval_set[i]].at(*it).y,
                        masks[eval_set[i]], 0) > 5){
                total_detected_false++;
            }
        }

    }

    delete snapshot_db;
*/

    FeatureMemory feature_memory;
    feature_memory.load("data/feature_memory_base.dat");
    for(unsigned i = 0; i < learn_set.size(); i++){
        for(unsigned j = 0; j < features[learn_set[i]].size(); j++){
            if(isObjectFeature(features[learn_set[i]].at(j), masks[learn_set[i]])){
                feature_memory.insertFeature(OBJECT_FEATURE, features[learn_set[i]].at(j), "");
            }
        }
    }
    feature_memory.rebuild();

    for (unsigned i = 0; i < eval_set.size(); i++) {
        //if((eval_set[i]-object_num*images_per_object)/4 == third){ continue; }

        for (unsigned j = 0; j < features[eval_set[i]].size(); j++) {
            std::string tmp;
            FeatureObjectType type =
                feature_memory.classifyFeature(features[eval_set[i]].at(j), APPROX_NN, tmp);

            if (isObjectFeature(features[eval_set[i]].at(j), masks[eval_set[i]])) {
                total_object_features++;
                if (type == OBJECT_FEATURE) {
                    total_detected_correct++;
                }
            }
            else if(type == OBJECT_FEATURE){
                total_detected_false++;
            }
        }
    }

    assert(num_ones < images_per_object);
    cdetected[num_ones].push_back((float)total_detected_correct/(float)total_object_features);
    fdetected[num_ones].push_back((float)total_detected_false/(float)total_object_features);

/*
    std::cout << "num learn: " << num_ones << std::endl;
    std::cout << total_detected_correct << " "
              << total_detected_false << " "
              << total_object_features << std::endl;
*/
    return 0.0f;
}

float MemoryValidator::edgeDist(int x, int y, IplImage *mask, int level){
    if(x < 0 || x >= mask->width || y < 0 || y > mask->height){
        return level;
    }

    if(level > 5 || cvGet2D(mask, y, x).val[0] > 128){
        return level;
    }

    return std::min<float>(std::min<float>(edgeDist(x-1,y,mask,level+1), edgeDist(x+1,y,mask,level+1)),
                           std::min<float>(edgeDist(x,y-1,mask,level+1), edgeDist(x,y+1,mask,level+1)));
}

unsigned MemoryValidator::numOnes(unsigned mask){
    unsigned result = 0;
    while(mask > 0){
        mask = mask & (mask-1);
        result++;
    }
    return result;
}

