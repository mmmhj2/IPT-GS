#include <ros/ros.h>
#include <SDL2/SDL.h>
#include <string>

#include "ColorSpaceCvt.h"

#ifdef BENCHMARK
#include <chrono>
#endif

namespace modulation
{
	SDL_Surface * mask;
	bool isLoad = false;
	
	// Check if the mask file is in correct format
	// 0x7F : Gray
	// 0xFF : White
	// 0x00 : Black
	void MaskSanityCheck()
	{
		if(mask->format->format != SDL_PIXELFORMAT_INDEX8)
		{
			ROS_ERROR("Wrong pixel format, expect %s, got %s", 
				SDL_GetPixelFormatName(SDL_PIXELFORMAT_INDEX8), 
				SDL_GetPixelFormatName(mask->format->format));
				return;
		}
		
		SDL_Color white, black, gray;
		white = mask->format->palette->colors[0xFF];
		black = mask->format->palette->colors[0x00];
		gray = mask->format->palette->colors[0x7F];
		
		if(white.r != 0xFF || white.g != 0xFF || white.b != 0xFF)
			ROS_WARN("Inconsistent color, expect white, got RGB(%d, %d, %d)", white.r, white.g, white.b);
		
		if(black.r != 0x00 || black.g != 0x00 || black.b != 0x00)
			ROS_WARN("Inconsistent color, expect black, got RGB(%d, %d, %d)", black.r, black.g, black.b);
		
		// Doesn't check gray because it is not needed
		//if(gray.r != 0x7F || gray.g != 0x7F || gray.b != 0x7F)
		//	ROS_WARN("Inconsistent color, expect gray, got RGB(%d, %d, %d)", gray.r, gray.g, gray.b);
	}
	
	
	void CleanUp()
	{
		ROS_DEBUG("Modulation module cleaning up...");
		if(mask != nullptr)
			SDL_FreeSurface(mask);
	}

	void LoadMaskMap(const std::string & path)
	{
		mask = SDL_LoadBMP(path.c_str());
		if(!mask)
		{
			ROS_FATAL("Cannot load mask file %s, %s", path.c_str() ,SDL_GetError());
			ros::shutdown();
		}
		isLoad = true;
		
		//ROS_INFO("Load mask bitmap, format %s", SDL_GetPixelFormatName(mask->format->format));
		MaskSanityCheck();
		if(SDL_MUSTLOCK(mask))
			ROS_WARN("Mask surface must be locked before accessing");
		
	}
	
	void Mask_Blk(cvt::byte * pixels, int length, int offset, int delta)
	{
		for(int index = 0; index < length; ++index)
		{
			if(((cvt::byte*)(mask->pixels))[offset + index] == 0x00)
			{
				int newL = (pixels)[(offset + index) * 4] + delta;
				newL = (newL > 255 ? 255 : newL);
				pixels[(offset + index) * 4] = (newL < 0 ? 0 : newL);
			}
		}
	}
	
	void CreateFrame(SDL_Surface * to, int delta = 5)
	{
		
		if(to->w != mask->w || to->h != mask->h)
			ROS_ERROR("Inconsistent surface size : mask %dx%d window %dx%d", mask->w, mask->h, to->w, to->h);
		
		if(SDL_MUSTLOCK(to))
			SDL_LockSurface(to);
		if(SDL_MUSTLOCK(mask))
			SDL_LockSurface(mask);

#ifdef BENCHMARK
		auto clock1 = std::chrono::steady_clock::now();
#endif
		
		cvt::BGR2CIELab((cvt::byte*)to->pixels, to->w * to->h * 4);

#ifdef BENCHMARK
		auto clock2 = std::chrono::steady_clock::now();
#endif		
		/*
		for(int index = 0; index < to->w * to->h; index++)
		{
			// masked byte
			if(((cvt::byte*)(mask->pixels))[index] == 0x00)
			{
				int newL = ((cvt::byte*)(to->pixels))[index * 4] + delta;
				newL = (newL > 255 ? 255 : newL);
				((cvt::byte*)(to->pixels))[index * 4] = (newL < 0 ? 0 : newL);
			}
		}
		*/
		int blocks = std::thread::hardware_concurrency() - 1;
		int tot_length = (to->w * to->h * 4);
		int blk_length = tot_length / blocks;
		blk_length = (blk_length >> 2) << 2;
		
		std::vector <std::future<void>> pools;
		for(int i = 0; i < blocks - 1; i++)
			pools.push_back(std::async(Mask_Blk, (cvt::byte*)to->pixels, blk_length, (blk_length * i) >> 2, delta));
		pools.push_back(std::async(
			Mask_Blk, (cvt::byte*)to->pixels,
			tot_length - blk_length * (blocks - 1),
			(blk_length * (blocks - 1)) >> 2,
			delta));
		
		std::for_each(pools.begin(), pools.end(), [](auto & task){task.wait();});

#ifdef BENCHMARK
		auto clock3 = std::chrono::steady_clock::now();
#endif
		
		cvt::CIELab2BGR((cvt::byte*)to->pixels, to->w * to->h * 4);
		
#ifdef BENCHMARK
		auto clock4 = std::chrono::steady_clock::now();
#endif
		
		if(SDL_MUSTLOCK(mask))
			SDL_LockSurface(mask);
		if(SDL_MUSTLOCK(to))
			SDL_UnlockSurface(to);

#ifdef BENCHMARK
		ROS_INFO_THROTTLE(1, "RGB -> CIELAB took %ld microseconds", 
			std::chrono::duration_cast<std::chrono::microseconds>(clock2 - clock1).count());
		ROS_INFO_THROTTLE(1, "Masking took %ld microseconds", 
			std::chrono::duration_cast<std::chrono::microseconds>(clock3 - clock2).count());
		ROS_INFO_THROTTLE(1, "CIELAB -> RGB took %ld microseconds", 
			std::chrono::duration_cast<std::chrono::microseconds>(clock4 - clock3).count());
#endif
		
	}
}
