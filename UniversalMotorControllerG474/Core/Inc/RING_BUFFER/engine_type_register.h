#ifndef TYPE_REGISTER_ENGINE_H
#define TYPE_REGISTER_ENGINE_H
#include <string>

template<typename T = void*> 
class TypeRegister
{

public:
      static uint8_t TYPE_ID;
      static int     GetTypeID()    { return TYPE_ID;}
      static size_t  GetTypeCount() { return TypeCount;}

      static bool isTypeValid()              { return GetTypeID() > 0;}
      static bool isTypeValid(const int& ID) { return ID <= TypeRegister<>::TypeCount;}

      static int RegisterType()
      { 
			    if(TYPE_ID > 0)   return TYPE_ID; // ALREADY REGISTERED
            TypeRegister<>::TypeCount++; TYPE_ID = TypeRegister<>::TypeCount;
            TypeRegister<>::SetTypeSize(sizeof(T));

            return TYPE_ID;
      }

      static uint16_t GetTypeSize()    {return sizeof(T);}
      static uint16_t GetMinTypeSize() {return TypeSizeMin;}
      static uint16_t GetMaxTypeSize() {return TypeSizeMax;}

      static void SetTypeSize(int size) 
      { 
            if(TypeSizeMin > size) TypeSizeMin = size; 
            if(TypeSizeMax < size) TypeSizeMax = size; 
      };

      static uint8_t TypeCount;
      static uint16_t TypeSizeMin;
      static uint16_t TypeSizeMax;
};

template<typename T> uint8_t TypeRegister<T>::TypeCount = 0;
template<typename T> uint16_t TypeRegister<T>::TypeSizeMin = 255;
template<typename T> uint16_t TypeRegister<T>::TypeSizeMax = 0;

template<typename T> uint8_t TypeRegister<T>::TYPE_ID = 0;


template<typename T>
class CommandDispatcher
{
public:
  typedef void (*processCallType)(T* Data);

  static processCallType processDispatch;
  static void setCallBack(processCallType process) { processDispatch = process; };
  static void processNop(T* Data) {};
};

template<typename T> CommandDispatcher<T>::processCallType CommandDispatcher<T>::processDispatch = nullptr;	;

#endif //TYPE_REGISTER_ENGINE_H
