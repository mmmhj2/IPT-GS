#include "ColorSpaceCvt.h"
#include <iostream>
using namespace std;

int main(int argc, char * argv[])
{
	if((argc - 1) % 3)
	{
		cout << "Wrong color format" << endl ;
		return 0;
	}
	
	//cvt::InitTable();
	
	unsigned char BGR[4];
	for(int i = 1; i < argc; i += 3)
	{
		int B, G, R;
		B = atoi(argv[i]);
		G = atoi(argv[i+1]);
		R = atoi(argv[i+2]);
		
		if(!(0 <= B && B <= 255))
		{
			cout << "Wrong color format at parameter blue" << endl ;
			return 0;
		}
		if(!(0 <= G && G <= 255))
		{
			cout << "Wrong color format at parameter green" << endl ;
			return 0;
		}
		if(!(0 <= R && R <= 255))
		{
			cout << "Wrong color format at parameter red" << endl ;
			return 0;
		}
		
		BGR[0] = B, BGR[1] = G, BGR[2] = R, BGR[3] = 0;
		
		cout << "B:" << (int)BGR[0] << " G:" << (int)BGR[1] << " R:" << (int)BGR[2] << "->" << endl ;
		
		cvt::BGR2CIELab((unsigned char *)BGR, 4);
		cout << "L:" << (int)BGR[0] << " A:" << (int)BGR[1] << " B:" << (int)BGR[2] << "->" << endl ;
		
		cvt::CIELab2BGR((unsigned char *)BGR, 4);
		cout << "B:" << (int)BGR[0] << " G:" << (int)BGR[1] << " R:" << (int)BGR[2] << endl ;
	}
	
}
