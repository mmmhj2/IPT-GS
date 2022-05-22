// Optimized color space conversion RGB <-> CIELAB
// Modified from the C-Sharp code in https://www.cnblogs.com/Imageshop/archive/2013/02/02/2889897.html

#ifndef _COLORSPACECVT_H_
#define _COLORSPACECVT_H_

#include <cmath>

namespace cvt
{
	constexpr float LABXRF = 0.433953f;
	constexpr float LABXGF = 0.376219f;
	constexpr float LABXBF = 0.189828f;
	constexpr float LABYRF = 0.212671f;
	constexpr float LABYGF = 0.715160f;
	constexpr float LABYBF = 0.072169f;
	constexpr float LABZRF = 0.017758f;
	constexpr float LABZGF = 0.109477f;
	constexpr float LABZBF = 0.872765f;

	constexpr float LABRXF = 3.0799327f;
	constexpr float LABRYF = -1.53715f;
	constexpr float LABRZF = -0.542782f;
	constexpr float LABGXF = -0.921235f;
	constexpr float LABGYF = 1.875991f;
	constexpr float LABGZF = 0.04524426f;
	constexpr float LABBXF = 0.0528909755f;
	constexpr float LABBYF = -0.204043f;
	constexpr float LABBZF = 1.15115158f;
	
	constexpr int Shift = 10;
	constexpr int HalfShiftValue = 1 << (Shift - 1);

	constexpr int LABXRI = (int)(LABXRF * (1 << Shift) + 0.5);
	constexpr int LABXGI = (int)(LABXGF * (1 << Shift) + 0.5);
	constexpr int LABXBI = (1 << Shift) - LABXRI - LABXGI;
	constexpr int LABYRI = (int)(LABYRF * (1 << Shift) + 0.5);
	constexpr int LABYGI = (int)(LABYGF * (1 << Shift) + 0.5);
	constexpr int LABYBI = (1 << Shift) - LABYRI - LABYGI;
	constexpr int LABZRI = (int)(LABZRF * (1 << Shift) + 0.5);
	constexpr int LABZGI = (int)(LABZGF * (1 << Shift) + 0.5);
	constexpr int LABZBI = (1 << Shift) - LABZRI - LABZGI;

	constexpr int LABRXI = (int)(LABRXF * (1 << Shift) + 0.5);
	constexpr int LABRYI = (int)(LABRYF * (1 << Shift) + 0.5);
	constexpr int LABRZI = (1 << Shift) - LABRXI - LABRYI;
	constexpr int LABGXI = (int)(LABGXF * (1 << Shift) + 0.5);
	constexpr int LABGYI = (int)(LABGYF * (1 << Shift) + 0.5);
	constexpr int LABGZI = (1 << Shift) - LABGXI - LABGYI;
	constexpr int LABBXI = (int)(LABBXF * (1 << Shift) + 0.5);
	constexpr int LABBYI = (int)(LABBYF * (1 << Shift) + 0.5);
	constexpr int LABBZI = (1 << Shift) - LABBXI - LABBYI;

	constexpr int ScaleLC = (int)(16 * 2.55 * (1 << Shift) + 0.5);
	constexpr int ScaleLT = (int)(116 * 2.55 + 0.5);

	constexpr int Threshold = (int)(6 * 6 * 6.0f / (29 * 29 * 29) * 255 * 4 + 0.5);

	constexpr float Div116 = 1.0f / 116 * (100.0F / 255);
	constexpr float Div500 = 1.0f / 500;
	constexpr float Div200 = 1.0f / 200;
	constexpr float Add16 = 16.0f / 116;
	constexpr float ThresoldF = 6.0f / 29;
	constexpr float MulT = 3 * (6 * 6.0f) / (29 * 29);
	constexpr float Sub4Div29 = 4.0f / 29;	
	
	using byte = unsigned char;
	
	int LabTab[1024 * 4];
	int TabX[256 * 256 * 4];
	int TabY[256 * 4];
	int TabZ[256 * 256 * 4];
	
	void InitTable()
	{
		int I, J, Index = 0;
		float X, Y, Z, T;
		for (I = 0; I < 1024; I++)
		{
			if (I > Threshold)
				LabTab[I] = (int)(pow((float)I / 1020, 1.0F / 3) * (1 << Shift) + 0.5);
			else
				LabTab[I] = (int)((29 * 29.0 * I / (6 * 6 * 3 * 1020) + 4.0 / 29) * (1 << Shift) + 0.5);
		}

		for (I = 0; I < 256; I++)
		{
			T = I * Div116 + Add16;
			if (T > ThresoldF)
				Y = T * T * T;
			else
				Y = MulT * (T - Sub4Div29);
			TabY[I] = (int)(Y * 255 + 0.5);
			for (J = 0; J < 256; J++)
			{
				X = T + Div500 * (J - 128);
				if (X > ThresoldF)
					X = X * X * X;
				else
					X = MulT * (X - Sub4Div29);
				TabX[Index] = (int)(X * 255 + 0.5);

				Z = T - Div200 * (J - 128);
				if (Z > ThresoldF)
					Z = Z * Z * Z;
				else
					Z = MulT * (Z - Sub4Div29);
				TabZ[Index] = (int)(Z * 255 + 0.5);
				Index++;
			}
		}
	}
	
	
	// Convert pixels from BGRA32 to CIELAB(L* a* b* A)
	void BGR2CIELab(byte * pixels, size_t length)
	{
		int X, Y, Z, L, A, B;
		int Blue, Green, Red;
		
		byte * ptr = (byte *)pixels;
		
		while(ptr - pixels < length)
		{
			Blue = ptr[0];
			Green = ptr[1];
			Red = ptr[2];
			// Ignore alpha channel
			//int Alpha = ptr[3];
			
			X = (Blue * LABXBI + Green * LABXGI + Red * LABXRI + HalfShiftValue) >> (Shift - 2);
			Y = (Blue * LABYBI + Green * LABYGI + Red * LABYRI + HalfShiftValue) >> (Shift - 2);
			Z = (Blue * LABZBI + Green * LABZGI + Red * LABZRI + HalfShiftValue) >> (Shift - 2);
			X = LabTab[X];
			Y = LabTab[Y];
			Z = LabTab[Z];
			L = ((ScaleLT * Y - ScaleLC + HalfShiftValue) >> Shift);
			A = ((500 * (X - Y) + HalfShiftValue) >> Shift) + 128;
			B = ((200 * (Y - Z) + HalfShiftValue) >> Shift) + 128;
			ptr[0] = (byte)(L & 0xFF);
			ptr[1] = (byte)(A & 0xFF);
			ptr[2] = (byte)(B & 0xFF);
			
			ptr += 4;
		}
		
	}
	
	void CIELab2BGR(byte * pixels, size_t length)
	{
		int X, Y, Z, L, A, B;
		int Blue, Green, Red;
		byte * ptr = (byte *)pixels;
		while(ptr - pixels < length)
		{
			L = ptr[0];
			A = ptr[1];
			B = ptr[2];
			
			X = TabX[L * 256 + A];
			Y = TabY[L];
			Z = TabZ[L * 256 + B];
			Blue = (X * LABBXI + Y * LABBYI + Z * LABBZI + HalfShiftValue) >> Shift;
			Green = (X * LABGXI + Y * LABGYI + Z * LABGZI + HalfShiftValue) >> Shift;
			Red = (X * LABRXI + Y * LABRYI + Z * LABRZI + HalfShiftValue) >> Shift;

			ptr[0] = (byte)(Blue & 0xFF);
			ptr[1] = (byte)(Green & 0xFF);
			ptr[2] = (byte)(Red & 0xFF);	
			
			ptr += 4;
		}
	}
	
}

#endif
