//#include "stdafx.h"
#include <windows.h>
#include "CppHelpers.h"
#include <stack>
#include <sstream>

#ifdef WIN32

namespace 
{
	static std::stack<COORD> stk;
}

#endif

std::ostream& LibIV::CppHelpers::Console::red(std::ostream & s)
{
	HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(h,FOREGROUND_RED|FOREGROUND_INTENSITY);
	return s;
}

std::ostream& LibIV::CppHelpers::Console::blue(std::ostream& s)
{
	HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(h,FOREGROUND_BLUE|FOREGROUND_INTENSITY);
	return s;
}

std::ostream& LibIV::CppHelpers::Console::green(std::ostream& s)
{
	HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(h,FOREGROUND_GREEN|FOREGROUND_INTENSITY);
	return s;
}

std::ostream& LibIV::CppHelpers::Console::gray(std::ostream& s)
{
	HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(h,FOREGROUND_INTENSITY);
	return s;
}

std::ostream& LibIV::CppHelpers::Console::white(std::ostream& s)
{
	HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(h,FOREGROUND_RED|FOREGROUND_GREEN|FOREGROUND_BLUE);
	return s;
}

std::ostream& LibIV::CppHelpers::Console::yellow(std::ostream& s)
{
	HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(h,FOREGROUND_GREEN | FOREGROUND_RED | FOREGROUND_INTENSITY);
	return s;
}

void LibIV::CppHelpers::Console::clearScreen()
{
	HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
	if(h!=INVALID_HANDLE_VALUE)
	{
		DWORD nCharWritten;
		COORD coordScreen = {0,0};
		CONSOLE_SCREEN_BUFFER_INFO csbi;
		DWORD dSize;
		
		if(GetConsoleScreenBufferInfo(h,&csbi) == 0)
			return;
		
		dSize = csbi.dwSize.X * csbi.dwSize.Y;

		if(FillConsoleOutputCharacter(h,' ',dSize,coordScreen,&nCharWritten) == 0)
			return;

		if(GetConsoleScreenBufferInfo(h,&csbi) == 0)
			return;

		if(FillConsoleOutputAttribute(h,csbi.wAttributes,dSize,coordScreen,&nCharWritten) == 0)
			return;

		if(SetConsoleCursorPosition(h,coordScreen) == 0)
			return;
	}
}

#ifdef WIN32
void LibIV::CppHelpers::Console::pushCursor()
{
	HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
	if(h!=INVALID_HANDLE_VALUE)
	{
		CONSOLE_SCREEN_BUFFER_INFO csbi;

		if(GetConsoleScreenBufferInfo(h,&csbi)==0)
			return;

		stk.push(csbi.dwCursorPosition);
	}
}

void LibIV::CppHelpers::Console::popCursor(bool clearLines /* = false */)
{
	HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
	if(h!=INVALID_HANDLE_VALUE)
	{
		CONSOLE_SCREEN_BUFFER_INFO csbi;
		COORD coord;
		DWORD lc,lo,cCharWritten;

		if(stk.empty())
			return;
		coord = stk.top();
		stk.pop();

		if(clearLines)
		{
			if(GetConsoleScreenBufferInfo(h,&csbi)==0)
				return;

			lo = coord.Y * csbi.dwSize.X + coord.X;
			lc = csbi.dwCursorPosition.Y * csbi.dwSize.X + csbi.dwCursorPosition.X;

			if(lc <= lo) 
				return;

			if(FillConsoleOutputCharacter(h,' ',lc-lo,coord,&cCharWritten)==0)
				return;

			if(GetConsoleScreenBufferInfo(h,&csbi)==0)
				return;

			if(FillConsoleOutputAttribute(h,csbi.wAttributes,lc-lo,coord,&cCharWritten)==0)
				return;

			if(SetConsoleCursorPosition(h,coord)==0)
				return;
		}
		else
		{
			if(SetConsoleCursorPosition(h,coord)==0)
				return;
		}
	}
}
#endif

std::string LibIV::CppHelpers::Global::intToStr(int n)
{
	std::ostringstream out;
	out<<n;	
	return out.str();
}

bool LibIV::CppHelpers::Global::readFlow(const char * filename, float *& u, float *& v, int & width, int & height, int store_type /* = 0 */)
{
	float tag;

	FILE* input = fopen(filename,"rb");

	if( input == 0 )
	{
		std::cout<<"Can not read Flow file : "<<filename<<std::endl;
		return false;
	}

	fread( &tag,  sizeof(float), 1, input );
	fread( &width,  sizeof(int), 1, input );
	fread( &height, sizeof(int), 1, input );

	float * buf = new float[width * height * 2];
	fread(buf,sizeof(float),width * height * 2,input);

	u = new float[width * height];
	v = new float[width * height];

	int idx = 0;
	for(int y = 0;y<height;y++)
		for(int x = 0;x<width;x++)
		{
			if(store_type)
			{
				// line by line
				u[y*width + x] = buf[idx++];
				v[y*width + x] = buf[idx++];
			}
			else
			{
				// column by column
				u[x*height + y] = buf[idx++];
				v[x*height + y] = buf[idx++];
			}
		}

	delete [] buf;
	fclose(input);
	return true;
}