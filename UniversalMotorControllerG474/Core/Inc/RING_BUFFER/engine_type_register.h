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
      static std::string Name;
      static std::string GetTypeName()              { return Name; }
      static std::string GetTypeName(const int& ID) { return Name; }

      static bool isTypeValid()              { return GetTypeID() > 0;}
      static bool isTypeValid(const int& ID) { return ID <= TypeRegister<>::TypeCount;}

      static int RegisterType(std::string TypeName = "NONE")
      { 
			    if(TYPE_ID > 0)   return TYPE_ID; // ALREADY REGISTERED
            Name = TypeName;
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


template<typename T> std::string TypeRegister<T>::Name = "NONE";

template<typename T> uint8_t TypeRegister<T>::TypeCount = 0;
template<typename T> uint16_t TypeRegister<T>::TypeSizeMin = 255;
template<typename T> uint16_t TypeRegister<T>::TypeSizeMax = 0;

template<typename T> uint8_t TypeRegister<T>::TYPE_ID = 0;

#endif //TYPE_REGISTER_ENGINE_H
