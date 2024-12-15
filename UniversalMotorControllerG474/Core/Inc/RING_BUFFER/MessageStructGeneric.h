#ifndef MESSAGE_STRUCT_GENERIC_H
#define MESSAGE_STRUCT_GENERIC_H


#include "TypeRegisterEngine.h"

template<typename T, typename H>
class MessageStructGeneric
{
  public:
    MessageStructGeneric();
	public:
    H HEADER;
    T DATA; 

  public:

	      bool isMessasge() { return (HEADER.isValid()); };
         int GetSize()    { return HEADER.DATA_SIZE + sizeof(H);};
         int GetSizeFromHeader(){ return HEADER.DATA_SIZE + sizeof(H);};
  static int GetSizeStatic() {return sizeof(T) + sizeof(uint8_t) + sizeof(H);};

  MessageStructGeneric<void*,H>& toGenericMessage() { return *reinterpret_cast<MessageStructGeneric<void*,H>*>(this);  }

};

template<typename T,typename H> 
MessageStructGeneric<T,H>::MessageStructGeneric()
{ 
  HEADER.DATA_SIZE = sizeof(T);
  HEADER.MESSAGE_IDENT = TypeRegister<T>::GetTypeID();
};


//==========================================================

#endif // MESSAGE_STRUCT_GENERIC_H