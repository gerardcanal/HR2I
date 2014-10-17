// stdafx.h: archivo de inclusión de los archivos de inclusión estándar del sistema
// o archivos de inclusión específicos de un proyecto utilizados frecuentemente,
// pero rara vez modificados
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>



// TODO: mencionar aquí los encabezados adicionales que el programa necesita

<<<<<<< HEAD

=======
>>>>>>> df9c394cbe3fa9e45481e905236bdccc9ee5ee01
// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
<<<<<<< HEAD
}
=======
}
>>>>>>> df9c394cbe3fa9e45481e905236bdccc9ee5ee01
