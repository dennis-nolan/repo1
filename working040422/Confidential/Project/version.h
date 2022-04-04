//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: appMain.H
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#ifndef __VERSION_H__
#define __VERSION_H__

//version 00_01
#define FW_VER 0x00002820//version 00.40

#ifndef SKIP_VERSION_CHECK
//check required module versions

//#include "SpiritVersion.h"
#define CRYPTO_VER 0x00000001
#if  (CRYPTO_VER != 0x00000001)
#error Incorrect Crypto Module version!
#endif

#include "SpiritVersion.h"
#if  (SPIRIT_VER != 0x0000000E)
//#if  (SPIRIT_VER != 0x00000003)
#error Incorrect Spirit Module version!
#endif

#include "CommonUtilsVersion.h"
#if COMUTILS_VER != 0x00000004
#error Incorrect Common Utilities version!
#endif

#include "CM2CoreVersion.h"
#if CM2CORE_VER != 0x0000001D
//#if CM2CORE_VER != 0x00000002
#error Incorrect CM2 Core Module version!
#endif

#include "appTestVersion.h"
#if  (APPTEST_VER != 0x00000009)
#error Incorrect APP test Module version!
#endif

#include "avengersCoreVersion.h"
#if  (AVENGERS_VER != 0x00000091)
#error Incorrect AVENGERS CORE Module version!
#endif
#endif


#endif

