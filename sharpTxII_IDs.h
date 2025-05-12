/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                             //
//                                              sharpTxII_IDs.h  ver1.0                                         //
//                                                                                                             //
//                                      Author: Mark Patten, Senior Engineer,                                  //
//                                                                                                             //
//                              C Firmware code for the remote switch receiver PCB Rev C                      //
//                                                                                                             //
//                                        Written for the PICC Compiler                                        //
//             (CCS Information Systems, www.ccsinfo.com, Brookfield, WI 53008, Tel. (262) 522-6500)           //
//                                                                                                             //
//                                     Program commencement date: Jan 2015                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define CUSTOMER_ID   0x55 // Hex address - unknown to customer, set by company to prevent cross-user activation
//#define CUSTOMER_ID   10 // Dec address - unknown to customer, set by company to prevent cross-user activation

#define RECEIVER_ID 345    // The address that must be sent in a command message to this receiver (or 999 for all receivers)
