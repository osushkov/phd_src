

#include <iostream>
#include <winsock2.h>

#include <digiclops.h>
#include <triclops.h>

#include <cv.h>
#include <cvaux.h>
#include <cxcore.h>
#include <highgui.h>

//#define PROCESS_STEREO
//#define CALIB_IMAGE 



//#define IMAGE_WIDTH		320
//#define IMAGE_HEIGHT	240

#define IMAGE_WIDTH		512
#define IMAGE_HEIGHT	384



#define SERVER_NAME     "humanoid-rightbrain.ai.cse.unsw.edu.au"
#define SERVER_PORT     9999

using namespace std;

DigiclopsContext    digiclopsContext;
TriclopsContext     triclopsContext;
TriclopsColorImage  leftColorImage, rightColorImage;
TriclopsInput       leftInput, rightInput;
TriclopsImage		triclopsImage;



#ifdef PROCESS_STEREO
TriclopsInput stereoInput;
TriclopsImage depthImage;
IplImage *depthIplImage;
#endif

#ifdef CALIB_IMAGE
TriclopsImage leftImage;
TriclopsImage rightImage;
#endif


IplImage *normLeftImage = 0;
IplImage *normRightImage = 0;
CvMat redMatrix, greenMatrix, blueMatrix;
CvMat leftMatrix, rightMatrix;

WORD wVersionRequested;
WSADATA wsaData;
int wsaerr;


int main()
{
	// to send image size
	char heightBuffer[4];
	char widthBuffer[5];

    // Camera Startup
    digiclopsCreateContext( &digiclopsContext );
    digiclopsInitialize( digiclopsContext, 0 );
#ifndef PROCESS_STEREO
	digiclopsSetImageTypes( digiclopsContext, LEFT_IMAGE | RIGHT_IMAGE );
#else
    digiclopsSetImageTypes( digiclopsContext, LEFT_IMAGE | RIGHT_IMAGE | STEREO_IMAGE );
#endif
	digiclopsSetImageResolution( digiclopsContext, DIGICLOPS_FULL );
    digiclopsStart( digiclopsContext );
    digiclopsGetTriclopsContextFromCamera( digiclopsContext, &triclopsContext );
	triclopsSetResolution(triclopsContext, IMAGE_HEIGHT, IMAGE_WIDTH);


    // OpenCV Startup
    normLeftImage  = cvCreateImage( cvSize(IMAGE_WIDTH, IMAGE_HEIGHT ), IPL_DEPTH_8U, 3);
	normRightImage = cvCreateImage( cvSize(IMAGE_WIDTH, IMAGE_HEIGHT ), IPL_DEPTH_8U, 3);



#ifdef PROCESS_STEREO
	depthIplImage = cvCreateImage( cvSize(IMAGE_WIDTH, IMAGE_HEIGHT ), IPL_DEPTH_8U, 1);
#endif

   // WinSock Startup  
    wVersionRequested = MAKEWORD(2, 2);
    wsaerr = WSAStartup(wVersionRequested, &wsaData);
    if (wsaerr != 0) {
        printf("The Winsock dll not found!\n");
        return 0;
    }
  
    if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2 ) {
        printf("The dll do not support the Winsock version %u.%u!\n", LOBYTE(wsaData.wVersion),HIBYTE(wsaData.wVersion));
        WSACleanup();
        return 0;
    }

    struct hostent *he;
    he = gethostbyname(SERVER_NAME);
    if( he == NULL){
       perror("gethostbyname");
       return 0;
    }

    // Socket
    SOCKET m_socket;
    m_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (m_socket == INVALID_SOCKET) {
        printf("Error at socket(): %ld\n", WSAGetLastError());
        WSACleanup();
        return 0;
    }

    // Bind
    sockaddr_in service;
    service.sin_family = AF_INET;
    service.sin_addr = *((struct in_addr*) he->h_addr_list[0]);
    service.sin_port = htons(9999);
    if (bind(m_socket, (SOCKADDR*)&service, sizeof(service)) == SOCKET_ERROR) {
        printf("bind() failed: %ld.\n", WSAGetLastError());
        closesocket(m_socket);
        return 0;
    }

    // Listen 
    if (listen( m_socket, 1) == SOCKET_ERROR) {
        printf("listen(): Error listening on socket %ld.\n", WSAGetLastError());
        return 0;
    }

    SOCKET AcceptSocket;
    int frameCount;
    int bytesSent;
    char key;
    char filename[10];



    
    while(1) {      // Server loop.
        AcceptSocket = SOCKET_ERROR;
        cout << "CameraServer: Awaiting connection from client." << endl;
        while (AcceptSocket == SOCKET_ERROR)
            AcceptSocket = accept(m_socket, NULL, NULL);
        cout << "CameraServer: Connected by client. Sending data." << endl;

        cvNamedWindow("Normal Left", CV_WINDOW_AUTOSIZE);
        cvMoveWindow("Normal Left", 0, 0);
        cvNamedWindow("Normal Right", CV_WINDOW_AUTOSIZE);
        cvMoveWindow("Normal Right", 350, 0);
#ifdef PROCESS_STEREO
		cvNamedWindow("Stereo", CV_WINDOW_AUTOSIZE);
#endif

		// send image size
		sprintf(heightBuffer, "%3d\0",normLeftImage->height);
		printf(heightBuffer);
		bytesSent = send(AcceptSocket, heightBuffer, 4, 0);
		if (bytesSent == SOCKET_ERROR) {
			cout << endl << "CameraServer: send() error " << WSAGetLastError() << ". Connection Terminated." << endl;
			return 0;
		}
		sprintf(widthBuffer, "%4d\0",normLeftImage->width);
		printf(widthBuffer	);
		bytesSent = send(AcceptSocket, widthBuffer, 5, 0);
	    if (bytesSent == SOCKET_ERROR) {
			cout << endl << "CameraServer: send() error " << WSAGetLastError() << ". Connection Terminated." << endl;
	        return 0;
	    }

        frameCount = 0;
        while(1) {  // Capture loop.
            digiclopsGrabImage( digiclopsContext);
            digiclopsExtractTriclopsInput( digiclopsContext, LEFT_IMAGE, &leftInput);
            digiclopsExtractTriclopsInput( digiclopsContext, RIGHT_IMAGE, &rightInput);

#ifdef PROCESS_STEREO
			digiclopsExtractTriclopsInput( digiclopsContext, STEREO_IMAGE, &stereoInput);
			triclopsPreprocess(triclopsContext, &stereoInput);
			triclopsStereo(triclopsContext);
			triclopsGetImage(triclopsContext, TriImg_DISPARITY, TriCam_REFERENCE, &depthImage);
			depthIplImage->imageData = (char*) depthImage.data;
			cvShowImage("Stereo", depthIplImage);           
#endif
			// Calibration : do not rectify image with triclops function
#ifndef CALIB_IMAGE
/*			int TRICLOPS_HEIGHT, TRICLOPS_WIDTH;

			triclopsGetResolution(triclopsContext, &TRICLOPS_HEIGHT, &TRICLOPS_WIDTH);
			cout << "Triclops Res: width " << TRICLOPS_WIDTH << ", height " << TRICLOPS_HEIGHT << endl;
*/			
			triclopsRectifyColorImage( triclopsContext, TriCam_LEFT, &leftInput, &leftColorImage);
            triclopsRectifyColorImage( triclopsContext, TriCam_RIGHT, &rightInput, &rightColorImage);

            cvInitMatHeader(&redMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, leftColorImage.red);
    	    cvInitMatHeader(&greenMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, leftColorImage.green);
	        cvInitMatHeader(&blueMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, leftColorImage.blue);
	        cvMerge(&blueMatrix, &greenMatrix, &redMatrix, NULL, normLeftImage);
		
	        cvInitMatHeader(&redMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, rightColorImage.red);
	        cvInitMatHeader(&greenMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, rightColorImage.green);
	        cvInitMatHeader(&blueMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, rightColorImage.blue);
 	        cvMerge(&blueMatrix, &greenMatrix, &redMatrix, NULL, normRightImage);
#else			
			if (leftInput.inputType == TriInp_RGB) {
                // RGB unpacked - unchecked
				cvInitMatHeader(&redMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, leftInput.u.rgb.red);
				cvInitMatHeader(&greenMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, leftInput.u.rgb.green);
				cvInitMatHeader(&blueMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, leftInput.u.rgb.blue);
				cvMerge(&blueMatrix, &greenMatrix, &redMatrix, NULL, normLeftImage);
			}
			else {
				// RGB packed
				// Extraction from TriclopsInputRGB32BitPacked
				int nrows = leftInput.nrows;
				int ncols = leftInput.ncols;
				int rowinc = leftInput.rowinc;			
				if(rowinc != 4*ncols) printf("Warning : extration of TriclopsInputRGB\n");

				char* red = (char*)malloc(nrows*ncols*(sizeof(char)));
				char* blue = (char*)malloc(nrows*ncols*(sizeof(char)));
				char* green = (char*)malloc(nrows*ncols*(sizeof(char)));
				char* data;
				data = (char*)leftInput.u.rgb32BitPacked.data;

				for(int y=0; y<nrows; y++)
					for(int x =0; x<ncols; x++) {
						blue[y*ncols+x] = data[y*rowinc+4*x];
						green[y*ncols+x] = data[y*rowinc+4*x+1];
						red[y*ncols+x] = data[y*rowinc+4*x+2];
					}

				cvInitMatHeader(&redMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, red);
				cvInitMatHeader(&greenMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, green);
				cvInitMatHeader(&blueMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, blue);
			    cvMerge(&blueMatrix, &greenMatrix, &redMatrix, NULL, normLeftImage);

				free(red);
				free(blue);
				free(green);
			}

			if (rightInput.inputType == TriInp_RGB) {
                // RGB unpacked - unchecked
				cvInitMatHeader(&redMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, rightInput.u.rgb.red);
				cvInitMatHeader(&greenMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, rightInput.u.rgb.green);
				cvInitMatHeader(&blueMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, rightInput.u.rgb.blue);
				cvMerge(&blueMatrix, &greenMatrix, &redMatrix, NULL, normRightImage);
			}
			else {
				// RGB packed
				// Extraction from TriclopsInputRGB32BitPacked
				int nrows = rightInput.nrows;
				int ncols = rightInput.ncols;
				int rowinc = rightInput.rowinc;			
				if(rowinc != 4*ncols) printf("Warning : extration of TriclopsInputRGB\n");

				unsigned char* red = (unsigned char*)malloc(nrows*ncols*(sizeof(unsigned char)));
				unsigned char* blue = (unsigned char*)malloc(nrows*ncols*(sizeof(unsigned char)));
				unsigned char* green = (unsigned char*)malloc(nrows*ncols*(sizeof(unsigned char)));
				unsigned char* data;
				data = (unsigned char*)rightInput.u.rgb32BitPacked.data;

				for(int y=0; y<nrows; y++)
					for(int x =0; x<ncols; x++) {
						blue[y*ncols+x] = data[y*rowinc+4*x];
						green[y*ncols+x] = data[y*rowinc+4*x+1];
						red[y*ncols+x] = data[y*rowinc+4*x+2];
					}

		        cvInitMatHeader(&redMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, red);
				cvInitMatHeader(&greenMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, green);
				cvInitMatHeader(&blueMatrix, IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, blue);
			    cvMerge(&blueMatrix, &greenMatrix, &redMatrix, NULL, normRightImage);
				
				free(red);
				free(blue);
				free(green);
			}
#endif
            cvShowImage("Normal Left", normLeftImage);
            cvShowImage("Normal Right", normRightImage);

            cout << "\rCameraServer: Sent " << ++frameCount << ".";

            bytesSent = send(AcceptSocket, normLeftImage->imageData, normLeftImage->imageSize, 0);
            if (bytesSent == SOCKET_ERROR) {
                cout << endl << "CameraServer: send() error " << WSAGetLastError() << ". Connection Terminated." << endl;
                break;
            }
            bytesSent = send(AcceptSocket, normRightImage->imageData, normRightImage->imageSize, 0);
            if (bytesSent == SOCKET_ERROR) {
                cout << endl << "CameraServer: send() error " << WSAGetLastError() << ". Connection Terminated." << endl;
                break;
            }
            
            key = cvWaitKey(1);
            if(key == 'q') {
                cout << endl << "CameraServer: Terminated by user." << endl;
                break;
            }
            if(key == 's') {
                sprintf(filename, "L%d.ppm", frameCount);
                cvSaveImage(filename, normLeftImage);
                sprintf(filename, "R%d.ppm", frameCount);
                cvSaveImage(filename, normRightImage);			
                cout << endl << "CameraServer: Images saved." << endl;
            }
        }
        closesocket(AcceptSocket);
        cvDestroyAllWindows();
    }

    cvReleaseImage(&normLeftImage);
    cvReleaseImage(&normRightImage);
#ifdef PROCESS_STEREO
	cvReleaseImage(&depthIplImage);
#endif
    digiclopsStop( digiclopsContext );
    digiclopsDestroyContext( digiclopsContext );
    triclopsDestroyContext( triclopsContext );

    return 0;
}