#ifndef DXL_H_
#define DXL_H_

#include <vector>
using namespace std;

class DXL;
class DXLChain {
protected :
	friend class DXL;
	int commPort;
	vector<DXL *> dxls;
	
public :
	DXLChain();
	bool open(const char *pathname);
	DXL *getDXL(int id);
	bool close();
};

class DXL {
protected :
	friend class DXLChain;
	DXLChain *chain;
	int id;
	
	DXL(DXLChain *chain, int id);
public :
	bool setResponse(int responseType);

	bool getPosn(int *posn);
	bool moveToPosn(int posn, int speed);
	bool getTorque(int *torqueEnabled);
	bool setTorque(int torqueEnabled);

    bool getLoad(int *load);
};

#endif /*DXL_H_*/
