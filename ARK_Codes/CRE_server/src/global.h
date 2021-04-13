#ifndef CRESEXP_GLOBAL_H
#define CRESEXP_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(CRESEXP_LIBRARY)
#  define CRESEXPSHARED_EXPORT Q_DECL_EXPORT
#else
#  define CRESEXPSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // CRESEXP_GLOBAL_H
