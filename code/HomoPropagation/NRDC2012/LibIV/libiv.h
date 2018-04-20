/*
	================================
	LibIV      2009/10/31
	================================

	Main include file
*/

#pragma  once
#include <LibIV/LibIVException.h>
#include <LibIV/Operation.h>
#include <LibIV/Tuple.h>
#include <LibIV/CppHelpers.h>
#include <LibIV/Array.h>
#include <LibIV/VideoContent.h>
#include <LibIV/VideoStream.h>


#ifndef LIBIV_SAFE_NAMESPACE
	
	using namespace LibIV;
	
	using namespace LibIV::CppHelpers;
	
	using namespace LibIV::Exception;
	
	using namespace LibIV::Math;
	
	using namespace LibIV::Operation;
	
	using namespace LibIV::Memory;
	using namespace LibIV::Memory::Array;
	
	using namespace LibIV::System;
	using namespace LibIV::System::Types;

	using namespace LibIV::Video;


#endif