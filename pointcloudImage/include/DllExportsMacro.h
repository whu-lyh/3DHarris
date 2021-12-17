#pragma once

#ifdef POINTCLOUDIMAGE_USE_AS_DLL

// The following ifdef block is the standard way of creating macros which make exporting
// from a DLL simpler. All files within this DLL are compiled with the POINTCLOUDIMAGE_EXPORT
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see
// POINTCLOUDIMAGE_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.

#ifdef POINTCLOUDIMAGE_EXPORT
#define POINTCLOUDIMAGE_API __declspec(dllexport)
#else
#define POINTCLOUDIMAGE_API __declspec(dllimport)
#endif

#else // NOT POINTCLOUDIMAGE_USE_AS_DLL

#define POINTCLOUDIMAGE_API

#endif // POINTCLOUDIMAGE_USE_AS_DLL

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif // PCL_NO_PRECOMPILE