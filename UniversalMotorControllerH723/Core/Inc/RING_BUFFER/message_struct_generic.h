#ifndef MESSAGEGENERIC_H
#define MESSAGEGENERIC_H

#include "engine_type_register.h"

template<typename T, typename H>
class MessageGeneric
{
  public:
    MessageGeneric();
	public:
    H HEADER;
    T DATA; 

  public:
	      bool isMessasge() { return (HEADER.isValid()); };
         int GetSize()    { return HEADER.DATA_SIZE + sizeof(H);};
         int GetSizeFromHeader(){ return HEADER.DATA_SIZE + sizeof(H);};
  static int GetSizeStatic() {return sizeof(T) + sizeof(H);};

  MessageGeneric<void*,H>& toGenericMessage() { return *reinterpret_cast<MessageGeneric<void*,H>*>(this);  }

};

template<typename T,typename H> 
MessageGeneric<T,H>::MessageGeneric()
{ 
  HEADER.DATA_SIZE = sizeof(T);
  HEADER.MESSAGE_IDENT = TypeRegister<T>::GetTypeID();
};


//==========================================================

#endif // MESSAGEGENERIC_H
