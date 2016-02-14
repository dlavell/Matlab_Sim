#include "__cf_PackageDeliverySim.h"
#include <math.h>
#include "PackageDeliverySim_acc.h"
#include "PackageDeliverySim_acc_private.h"
#include <stdio.h>
#include "simstruc.h"
#include "fixedpoint.h"
#define CodeFormat S-Function
#define AccDefine1 Accelerator_S-Function
real_T rt_urand_Upu32_Yd_f_pw_snf ( uint32_T * u ) { uint32_T lo ; uint32_T
hi ; lo = * u % 127773U * 16807U ; hi = * u / 127773U * 2836U ; if ( lo < hi
) { * u = 2147483647U - ( hi - lo ) ; } else { * u = lo - hi ; } return (
real_T ) * u * 4.6566128752457969E-10 ; } static void mdlOutputs ( SimStruct
* S , int_T tid ) { real_T * lastU ; real_T lastTime ; boolean_T p0nf2eyajo ;
idlexrjgpo * _rtB ; hbdftv5mfu * _rtP ; p14iuulsws * _rtX ; peas4oso0k *
_rtDW ; _rtDW = ( ( peas4oso0k * ) ssGetRootDWork ( S ) ) ; _rtX = ( (
p14iuulsws * ) ssGetContStates ( S ) ) ; _rtP = ( ( hbdftv5mfu * )
ssGetDefaultParam ( S ) ) ; _rtB = ( ( idlexrjgpo * ) _ssGetBlockIO ( S ) ) ;
ssCallAccelRunBlock ( S , 6 , 0 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock
( S , 6 , 1 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 6 , 2 ,
SS_CALL_MDL_OUTPUTS ) ; if ( ssIsSampleHit ( S , 1 , 0 ) ) { _rtB ->
culamumh0e [ 0 ] = _rtP -> P_0 [ 0 ] ; _rtB -> culamumh0e [ 1 ] = _rtP -> P_0
[ 1 ] ; _rtB -> culamumh0e [ 2 ] = _rtP -> P_0 [ 2 ] ; _rtB -> itxoyuevcy =
_rtP -> P_1 ; _rtB -> cu2e2uwkad = _rtP -> P_2 ; _rtB -> ds1kkl4355 = _rtP ->
P_3 ; _rtB -> awkh12jurt = _rtP -> P_4 ; _rtB -> pmtxpvprhg = _rtP -> P_5 ;
ssCallAccelRunBlock ( S , 3 , 0 , SS_CALL_MDL_OUTPUTS ) ; _rtB -> k0ljbl0pfv
= _rtP -> P_6 ; ssCallAccelRunBlock ( S , 4 , 0 , SS_CALL_MDL_OUTPUTS ) ; }
ssCallAccelRunBlock ( S , 5 , 0 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock
( S , 6 , 13 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 6 , 14 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 6 , 15 ,
SS_CALL_MDL_OUTPUTS ) ; if ( ssIsSampleHit ( S , 1 , 0 ) ) { _rtB ->
grf4h2renl = _rtP -> P_7 ; } ssCallAccelRunBlock ( S , 1 , 0 ,
SS_CALL_MDL_OUTPUTS ) ; p0nf2eyajo = ( ( _rtB -> jf5cnzjkvo != 0.0 ) && ( ! (
_rtB -> aypj1x4oja != 0.0 ) ) ) ; if ( ssIsSampleHit ( S , 1 , 0 ) ) { _rtB
-> b3c15jgt0z = ( _rtDW -> j4basf1zpu >= _rtP -> P_11 ) ; _rtB -> pih4yw5usq
= _rtP -> P_12 ; } switch ( ( int32_T ) _rtB -> grf4h2renl ) { case 1 : _rtB
-> mku4wnsjsx = _rtB -> jf5cnzjkvo ; break ; case 2 : _rtB -> mku4wnsjsx =
p0nf2eyajo ; break ; case 3 : _rtB -> mku4wnsjsx = ( p0nf2eyajo || ( ( _rtB
-> jf5cnzjkvo != 0.0 ) && ( _rtB -> aypj1x4oja != 0.0 ) && _rtB -> b3c15jgt0z
) ) ; break ; default : _rtB -> mku4wnsjsx = _rtB -> pih4yw5usq ; break ; }
if ( ssIsSampleHit ( S , 1 , 0 ) ) { _rtB -> faolbilbtg = _rtP -> P_13 ; } if
( ( _rtDW -> oarj5vv3y3 == ( rtMinusInf ) ) || ( _rtDW -> oarj5vv3y3 ==
ssGetTaskTime ( S , 0 ) ) ) { _rtDW -> oarj5vv3y3 = ssGetTaskTime ( S , 0 ) ;
memcpy ( & _rtB -> kglecikwkb [ 0 ] , & _rtP -> P_14 [ 0 ] , 19U * sizeof (
real_T ) ) ; } else { _rtB -> kglecikwkb [ 0 ] = _rtX -> eeo550jiw0 [ 0 ] ;
_rtB -> kglecikwkb [ 1 ] = _rtX -> eeo550jiw0 [ 1 ] + _rtB -> faolbilbtg ;
memcpy ( & _rtB -> kglecikwkb [ 2 ] , & _rtB -> c00lhnglkx [ 0 ] , 17U *
sizeof ( real_T ) ) ; } if ( ssIsMajorTimeStep ( S ) ) { ZCEventType zcEvent
; boolean_T resetIntg = false ; zcEvent = rt_ZCFcn ( ANY_ZERO_CROSSING , & (
( ebtqtlmvbs * ) _ssGetPrevZCSigState ( S ) ) -> d03ndqbhnu , _rtB ->
mku4wnsjsx ) ; if ( _rtB -> mku4wnsjsx != 0.0 || _rtDW -> jmgtnqu2of .
IcNeedsLoading ) { resetIntg = true ; { int_T i1 ; real_T * xc = & ( (
p14iuulsws * ) ssGetContStates ( S ) ) -> eeo550jiw0 [ 0 ] ; const real_T *
u2 = & _rtB -> kglecikwkb [ 0 ] ; for ( i1 = 0 ; i1 < 19 ; i1 ++ ) { xc [ i1
] = u2 [ i1 ] ; } } } else { if ( zcEvent ) resetIntg = true ; } if (
resetIntg ) { ssSetSolverNeedsReset ( S ) ; ssSetBlkStateChange ( S ) ; } } {
int_T i1 ; real_T * y0 = & _rtB -> hm0gk0gmdb [ 0 ] ; real_T * xc = & ( (
p14iuulsws * ) ssGetContStates ( S ) ) -> eeo550jiw0 [ 0 ] ; for ( i1 = 0 ;
i1 < 19 ; i1 ++ ) { y0 [ i1 ] = xc [ i1 ] ; } } if ( ( _rtDW -> j4havqmcbz ==
( rtMinusInf ) ) || ( _rtDW -> j4havqmcbz == ssGetTaskTime ( S , 0 ) ) ) {
_rtDW -> j4havqmcbz = ssGetTaskTime ( S , 0 ) ; memcpy ( & _rtB -> nwj0enpthd
[ 0 ] , & _rtP -> P_15 [ 0 ] , 17U * sizeof ( real_T ) ) ; } else { memcpy (
& _rtB -> nwj0enpthd [ 0 ] , & _rtB -> hm0gk0gmdb [ 2 ] , 17U * sizeof (
real_T ) ) ; } if ( ssIsSampleHit ( S , 1 , 0 ) ) { _rtB -> bgwfk0alqt = _rtP
-> P_17 * _rtP -> P_16 ; _rtB -> l14bwhwzcc = _rtP -> P_18 * _rtP -> P_16 ;
_rtB -> dwnpyfslq3 = _rtP -> P_19 * _rtP -> P_16 ; _rtB -> kadx2blx43 = _rtP
-> P_20 * _rtP -> P_16 ; _rtB -> pdv4mhew0w = _rtP -> P_21 * _rtP -> P_16 ;
_rtB -> fuznxoqta0 = _rtP -> P_22 * _rtP -> P_16 ; } ssCallAccelRunBlock ( S
, 2 , 0 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 6 , 39 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 6 , 40 ,
SS_CALL_MDL_OUTPUTS ) ; if ( ( _rtDW -> d2omkb40lq >= ssGetT ( S ) ) && (
_rtDW -> pjnqppg0zc >= ssGetT ( S ) ) ) { _rtB -> lic3k04pyz = 0.0 ; } else {
lastTime = _rtDW -> d2omkb40lq ; lastU = & _rtDW -> ia4c4krcdk ; if ( _rtDW
-> d2omkb40lq < _rtDW -> pjnqppg0zc ) { if ( _rtDW -> pjnqppg0zc < ssGetT ( S
) ) { lastTime = _rtDW -> pjnqppg0zc ; lastU = & _rtDW -> kckg1mt4pt ; } }
else { if ( _rtDW -> d2omkb40lq >= ssGetT ( S ) ) { lastTime = _rtDW ->
pjnqppg0zc ; lastU = & _rtDW -> kckg1mt4pt ; } } _rtB -> lic3k04pyz = ( _rtB
-> bvmjwbthkj - * lastU ) / ( ssGetT ( S ) - lastTime ) ; }
ssCallAccelRunBlock ( S , 6 , 42 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 6 , 43 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 6 , 44 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 6 , 45 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 6 , 46 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 6 , 47 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 6 , 48 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 6 , 49 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 6 , 50 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 6 , 51 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 6 , 52 , SS_CALL_MDL_OUTPUTS ) ; if ( ssIsSampleHit
( S , 1 , 0 ) ) { _rtB -> bnro1jon2p = _rtP -> P_23 ; _rtB -> e44cnsd5le =
_rtP -> P_24 ; } _rtB -> copjkuykwu = ( ( _rtB -> hm0gk0gmdb [ 0 ] >= _rtB ->
bnro1jon2p ) || ( _rtB -> hm0gk0gmdb [ 1 ] >= _rtB -> e44cnsd5le ) || ( ! ( (
_rtB -> aypj1x4oja != 0.0 ) || ( _rtB -> jf5cnzjkvo != 0.0 ) ) ) ) ; if (
ssIsSampleHit ( S , 1 , 0 ) ) { if ( _rtB -> copjkuykwu ) {
ssSetStopRequested ( S , 1 ) ; } _rtB -> pm4o0k2cz3 = _rtP -> P_25 ; _rtB ->
o1jn2bcbbu = _rtP -> P_26 ; } UNUSED_PARAMETER ( tid ) ; }
#define MDL_UPDATE
static void mdlUpdate ( SimStruct * S , int_T tid ) { real_T * lastU ;
idlexrjgpo * _rtB ; hbdftv5mfu * _rtP ; p14iuulsws * _rtX ; peas4oso0k *
_rtDW ; _rtDW = ( ( peas4oso0k * ) ssGetRootDWork ( S ) ) ; _rtX = ( (
p14iuulsws * ) ssGetContStates ( S ) ) ; _rtP = ( ( hbdftv5mfu * )
ssGetDefaultParam ( S ) ) ; _rtB = ( ( idlexrjgpo * ) _ssGetBlockIO ( S ) ) ;
ssCallAccelRunBlock ( S , 6 , 0 , SS_CALL_MDL_UPDATE ) ; if ( ssIsSampleHit (
S , 1 , 0 ) ) { _rtDW -> j4basf1zpu = ( _rtP -> P_9 - _rtP -> P_8 ) *
rt_urand_Upu32_Yd_f_pw_snf ( & _rtDW -> gznusywyhy ) + _rtP -> P_8 ; } _rtDW
-> jmgtnqu2of . IcNeedsLoading = 0 ; if ( _rtDW -> d2omkb40lq == ( rtInf ) )
{ _rtDW -> d2omkb40lq = ssGetT ( S ) ; lastU = & _rtDW -> ia4c4krcdk ; } else
if ( _rtDW -> pjnqppg0zc == ( rtInf ) ) { _rtDW -> pjnqppg0zc = ssGetT ( S )
; lastU = & _rtDW -> kckg1mt4pt ; } else if ( _rtDW -> d2omkb40lq < _rtDW ->
pjnqppg0zc ) { _rtDW -> d2omkb40lq = ssGetT ( S ) ; lastU = & _rtDW ->
ia4c4krcdk ; } else { _rtDW -> pjnqppg0zc = ssGetT ( S ) ; lastU = & _rtDW ->
kckg1mt4pt ; } * lastU = _rtB -> bvmjwbthkj ; UNUSED_PARAMETER ( tid ) ; }
#define MDL_DERIVATIVES
static void mdlDerivatives ( SimStruct * S ) { idlexrjgpo * _rtB ; p14iuulsws
* _rtX ; peas4oso0k * _rtDW ; _rtDW = ( ( peas4oso0k * ) ssGetRootDWork ( S )
) ; _rtX = ( ( p14iuulsws * ) ssGetContStates ( S ) ) ; _rtB = ( ( idlexrjgpo
* ) _ssGetBlockIO ( S ) ) ; ssCallAccelRunBlock ( S , 6 , 0 ,
SS_CALL_MDL_DERIVATIVES ) ; { if ( _rtB -> mku4wnsjsx == 0.0 ) { ( (
eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 0 ] = _rtB -> pm4o0k2cz3 ; }
else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 0 ] = 0.0 ; } if (
_rtB -> mku4wnsjsx == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) ->
eeo550jiw0 [ 1 ] = _rtB -> o1jn2bcbbu ; } else { ( ( eng5ve0b2q * ) ssGetdX (
S ) ) -> eeo550jiw0 [ 1 ] = 0.0 ; } if ( _rtB -> mku4wnsjsx == 0.0 ) { ( (
eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 2 ] = _rtB -> h0gw5goxcw [ 0 ]
; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 2 ] = 0.0 ; }
if ( _rtB -> mku4wnsjsx == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) ->
eeo550jiw0 [ 3 ] = _rtB -> h0gw5goxcw [ 1 ] ; } else { ( ( eng5ve0b2q * )
ssGetdX ( S ) ) -> eeo550jiw0 [ 3 ] = 0.0 ; } if ( _rtB -> mku4wnsjsx == 0.0
) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 4 ] = _rtB ->
h0gw5goxcw [ 2 ] ; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0
[ 4 ] = 0.0 ; } if ( _rtB -> mku4wnsjsx == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX
( S ) ) -> eeo550jiw0 [ 5 ] = _rtB -> gy005zvtb2 [ 0 ] ; } else { ( (
eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 5 ] = 0.0 ; } if ( _rtB ->
mku4wnsjsx == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 6 ]
= _rtB -> gy005zvtb2 [ 1 ] ; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) ->
eeo550jiw0 [ 6 ] = 0.0 ; } if ( _rtB -> mku4wnsjsx == 0.0 ) { ( ( eng5ve0b2q
* ) ssGetdX ( S ) ) -> eeo550jiw0 [ 7 ] = _rtB -> gy005zvtb2 [ 2 ] ; } else {
( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 7 ] = 0.0 ; } if ( _rtB ->
mku4wnsjsx == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 8 ]
= _rtB -> an4mvfc3ok [ 0 ] ; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) ->
eeo550jiw0 [ 8 ] = 0.0 ; } if ( _rtB -> mku4wnsjsx == 0.0 ) { ( ( eng5ve0b2q
* ) ssGetdX ( S ) ) -> eeo550jiw0 [ 9 ] = _rtB -> an4mvfc3ok [ 1 ] ; } else {
( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 9 ] = 0.0 ; } if ( _rtB ->
mku4wnsjsx == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 10 ]
= _rtB -> an4mvfc3ok [ 2 ] ; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) ->
eeo550jiw0 [ 10 ] = 0.0 ; } if ( _rtB -> mku4wnsjsx == 0.0 ) { ( ( eng5ve0b2q
* ) ssGetdX ( S ) ) -> eeo550jiw0 [ 11 ] = _rtB -> j3b4qdjnfh [ 0 ] ; } else
{ ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 11 ] = 0.0 ; } if ( _rtB
-> mku4wnsjsx == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [
12 ] = _rtB -> j3b4qdjnfh [ 1 ] ; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) )
-> eeo550jiw0 [ 12 ] = 0.0 ; } if ( _rtB -> mku4wnsjsx == 0.0 ) { ( (
eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 13 ] = _rtB -> j3b4qdjnfh [ 2
] ; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 13 ] = 0.0 ;
} if ( _rtB -> mku4wnsjsx == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) ->
eeo550jiw0 [ 14 ] = _rtB -> pachzqrb0b ; } else { ( ( eng5ve0b2q * ) ssGetdX
( S ) ) -> eeo550jiw0 [ 14 ] = 0.0 ; } if ( _rtB -> mku4wnsjsx == 0.0 ) { ( (
eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 15 ] = _rtB -> n5ghauyxs1 [ 0
] ; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 15 ] = 0.0 ;
} if ( _rtB -> mku4wnsjsx == 0.0 ) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) ->
eeo550jiw0 [ 16 ] = _rtB -> n5ghauyxs1 [ 1 ] ; } else { ( ( eng5ve0b2q * )
ssGetdX ( S ) ) -> eeo550jiw0 [ 16 ] = 0.0 ; } if ( _rtB -> mku4wnsjsx == 0.0
) { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 17 ] = _rtB ->
n5ghauyxs1 [ 2 ] ; } else { ( ( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0
[ 17 ] = 0.0 ; } if ( _rtB -> mku4wnsjsx == 0.0 ) { ( ( eng5ve0b2q * )
ssGetdX ( S ) ) -> eeo550jiw0 [ 18 ] = _rtB -> n5ghauyxs1 [ 3 ] ; } else { (
( eng5ve0b2q * ) ssGetdX ( S ) ) -> eeo550jiw0 [ 18 ] = 0.0 ; } } } static
void mdlInitializeSizes ( SimStruct * S ) { ssSetChecksumVal ( S , 0 ,
466068284U ) ; ssSetChecksumVal ( S , 1 , 906048057U ) ; ssSetChecksumVal ( S
, 2 , 2319792655U ) ; ssSetChecksumVal ( S , 3 , 4194681732U ) ; { mxArray *
slVerStructMat = NULL ; mxArray * slStrMat = mxCreateString ( "simulink" ) ;
char slVerChar [ 10 ] ; int status = mexCallMATLAB ( 1 , & slVerStructMat , 1
, & slStrMat , "ver" ) ; if ( status == 0 ) { mxArray * slVerMat = mxGetField
( slVerStructMat , 0 , "Version" ) ; if ( slVerMat == NULL ) { status = 1 ; }
else { status = mxGetString ( slVerMat , slVerChar , 10 ) ; } }
mxDestroyArray ( slStrMat ) ; mxDestroyArray ( slVerStructMat ) ; if ( (
status == 1 ) || ( strcmp ( slVerChar , "8.4" ) != 0 ) ) { return ; } }
ssSetOptions ( S , SS_OPTION_EXCEPTION_FREE_CODE ) ; if ( ssGetSizeofDWork (
S ) != sizeof ( peas4oso0k ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal DWork sizes do "
"not match for accelerator mex file." ) ; } if ( ssGetSizeofGlobalBlockIO ( S
) != sizeof ( idlexrjgpo ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal BlockIO sizes do "
"not match for accelerator mex file." ) ; } { int ssSizeofParams ;
ssGetSizeofParams ( S , & ssSizeofParams ) ; if ( ssSizeofParams != sizeof (
hbdftv5mfu ) ) { static char msg [ 256 ] ; sprintf ( msg ,
"Unexpected error: Internal Parameters sizes do "
"not match for accelerator mex file." ) ; } } _ssSetDefaultParam ( S , (
real_T * ) & krmhsmofha ) ; rt_InitInfAndNaN ( sizeof ( real_T ) ) ; } static
void mdlInitializeSampleTimes ( SimStruct * S ) { { SimStruct * childS ;
SysOutputFcn * callSysFcns ; childS = ssGetSFunction ( S , 0 ) ; callSysFcns
= ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 1 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 2 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 3 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 4 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; } } static void mdlTerminate ( SimStruct * S ) { }
#include "simulink.c"
