
#ifndef _MainController_H_
#define _MainController_H_


#include "Features/SIFTGenerator.h"
#include "Features/StereoFeatureCorrelation.h"
#include "Visualisation/SceneRenderer.h"

#include <string>

class MainController {
  public:
    static MainController& instance(void);

    void initialise(void);
    void buildObject(std::string object_name);

  private:

    MainController();
};

#endif

