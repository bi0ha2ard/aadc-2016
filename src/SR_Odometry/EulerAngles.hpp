/**** EulerAngles.c - Convert Euler angles to/from matrix or quat ****/
/* Ken Shoemake, 1993 */
/**
http://www.realtimerendering.com/resources/GraphicsGems/

LICENSE

This code repository predates the concept of Open Source, and predates most licenses along such lines. As such, the official license truly is:

EULA: The Graphics Gems code is copyright-protected. In other words, you cannot claim the text of the code as your own and resell it. Using the code is permitted in any program, product, or library, non-commercial or commercial. Giving credit is not required, though is a nice gesture. The code comes as-is, and if there are any flaws or problems with any Gems code, nobody involved with Gems - authors, editors, publishers, or webmasters - are to be held responsible. Basically, don't be a jerk, and remember that anything free comes with no guarantee.

 **/

/**** EulerAngles.h - Support for 24 angle schemes ****/
/* source code from "Graphic Germs IV" von Ken Shoemake, 1993 */
#ifndef _H_EulerAngles
#define _H_EulerAngles
#include "QuatTypes.h"
/*** Order type constants, constructors, extractors ***/
    /* There are 24 possible conventions, designated by:    */
    /*	  o EulAxI = axis used initially		    */
    /*	  o EulPar = parity of axis permutation		    */
    /*	  o EulRep = repetition of initial axis as last	    */
    /*	  o EulFrm = frame from which axes are taken	    */
    /* Axes I,J,K will be a permutation of X,Y,Z.	    */
    /* Axis H will be either I or K, depending on EulRep.   */
    /* Frame S takes axes from initial static frame.	    */
    /* If ord = (AxI=X, Par=Even, Rep=No, Frm=S), then	    */
    /* {a,b,c,ord} means Rz(c)Ry(b)Rx(a), where Rz(c)v	    */
    /* rotates v around Z by c radians.			    */
#define EulFrmS	     0
#define EulFrmR	     1
#define EulFrm(ord)  ((unsigned)(ord)&1)
#define EulRepNo     0
#define EulRepYes    1
#define EulRep(ord)  (((unsigned)(ord)>>1)&1)
#define EulParEven   0
#define EulParOdd    1
#define EulPar(ord)  (((unsigned)(ord)>>2)&1)
#define EulSafe	     "\000\001\002\000"
#define EulNext	     "\001\002\000\001"
#define EulAxI(ord)  ((int)(EulSafe[(((unsigned)(ord)>>3)&3)]))
#define EulAxJ(ord)  ((int)(EulNext[EulAxI(ord)+(EulPar(ord)==EulParOdd)]))
#define EulAxK(ord)  ((int)(EulNext[EulAxI(ord)+(EulPar(ord)!=EulParOdd)]))
#define EulAxH(ord)  ((EulRep(ord)==EulRepNo)?EulAxK(ord):EulAxI(ord))
    /* EulGetOrd unpacks all useful information about order simultaneously. */
#define EulGetOrd(ord,i,j,k,h,n,s,f) {unsigned o=ord;f=o&1;o>>=1;s=o&1;o>>=1;\
    n=o&1;o>>=1;i=EulSafe[o&3];j=EulNext[i+n];k=EulNext[i+1-n];h=s?k:i;}
    /* EulOrd creates an order value between 0 and 23 from 4-tuple choices. */
#define EulOrd(i,p,r,f)	   (((((((i)<<1)+(p))<<1)+(r))<<1)+(f))
    /* Static axes */
#define EulOrdXYZs    EulOrd(_X,EulParEven,EulRepNo,EulFrmS)
#define EulOrdXYXs    EulOrd(_X,EulParEven,EulRepYes,EulFrmS)
#define EulOrdXZYs    EulOrd(_X,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdXZXs    EulOrd(_X,EulParOdd,EulRepYes,EulFrmS)
#define EulOrdYZXs    EulOrd(_Y,EulParEven,EulRepNo,EulFrmS)
#define EulOrdYZYs    EulOrd(_Y,EulParEven,EulRepYes,EulFrmS)
#define EulOrdYXZs    EulOrd(_Y,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdYXYs    EulOrd(_Y,EulParOdd,EulRepYes,EulFrmS)
#define EulOrdZXYs    EulOrd(_Z,EulParEven,EulRepNo,EulFrmS)
#define EulOrdZXZs    EulOrd(_Z,EulParEven,EulRepYes,EulFrmS)
#define EulOrdZYXs    EulOrd(_Z,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdZYZs    EulOrd(_Z,EulParOdd,EulRepYes,EulFrmS)
    /* Rotating axes */
#define EulOrdZYXr    EulOrd(_X,EulParEven,EulRepNo,EulFrmR)
#define EulOrdXYXr    EulOrd(_X,EulParEven,EulRepYes,EulFrmR)
#define EulOrdYZXr    EulOrd(_X,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdXZXr    EulOrd(_X,EulParOdd,EulRepYes,EulFrmR)
#define EulOrdXZYr    EulOrd(_Y,EulParEven,EulRepNo,EulFrmR)
#define EulOrdYZYr    EulOrd(_Y,EulParEven,EulRepYes,EulFrmR)
#define EulOrdZXYr    EulOrd(_Y,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdYXYr    EulOrd(_Y,EulParOdd,EulRepYes,EulFrmR)
#define EulOrdYXZr    EulOrd(_Z,EulParEven,EulRepNo,EulFrmR)
#define EulOrdZXZr    EulOrd(_Z,EulParEven,EulRepYes,EulFrmR)
#define EulOrdXYZr    EulOrd(_Z,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdZYZr    EulOrd(_Z,EulParOdd,EulRepYes,EulFrmR)

EulerAngles Eul_(float ai, float aj, float ah, int order);
Quat Eul_ToQuat(EulerAngles ea);
void Eul_ToHMatrix(EulerAngles ea, HMatrix M);
EulerAngles Eul_FromHMatrix(HMatrix M, int order);
EulerAngles Eul_FromQuat(Quat q, int order);
#endif
