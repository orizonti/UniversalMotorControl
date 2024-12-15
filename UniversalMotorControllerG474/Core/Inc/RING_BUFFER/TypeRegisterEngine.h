#ifndef TYPE_REGISTER_ENGINE_H
#define TYPE_REGISTER_ENGINE_H
#include "CommandStructures.h"
#include <string>
#include <map>
#include <typeinfo>
#include <cstdio>
#include "main.h"

template<typename T> class TypeRegister;

class TypeTagRegister
{
//utilite none template class to stroe TypesID map
public:
      template<typename TT> class TYPE_TAG{};
      template<typename TT> static int GetTypeID(TYPE_TAG<TT>) { return TypeRegister<TT>::GetTypeID();}
      template<typename TT> static std::string GetTypeName(TYPE_TAG<TT>) { return TypeRegister<TT>::GetTypeName(); }

                            static std::string GetTypeName(const int& ID) {if(isTypeValid(ID)) return TypesID[ID]; 
                                                                                  return "ANVALID_TYPE"; }
      template<typename TT> 
      static bool isTypeValid(const TYPE_TAG<TT>& TAG) { return TypesID.contains(TypeRegister<TT>::GetTypeID());}
      static bool isTypeValid(const int& ID)            { return TypesID.contains(ID);}

      //static bool isTypeValid(const TYPE_TAG<TT>& TAG) { return true;}
      //static bool isTypeValid(const int& ID)            { return true;}

      static size_t GetTypeCount() { return TypesID.size(); }

      template<typename TT> static size_t GetTypeSize(const TYPE_TAG<TT>& TAG) { return TypesSizes[GetTypeID(TAG)];};
      static size_t GetMinTypeSize() { return MinTypeSize;};
      static size_t GetMaxTypeSize() { return MaxTypeSize;};

      template<typename TT> static void RegisterType(const TYPE_TAG<TT>& TAG) 
      { 
        TypesID[GetTypeID(TAG)] = GetTypeName(TAG);
        TypesSizes[GetTypeID(TAG)] = sizeof(TT);

        if(sizeof(TT) < MinTypeSize || MinTypeSize == 0) MinTypeSize = sizeof(TT);
        if(sizeof(TT) > MaxTypeSize || MaxTypeSize == 0) MaxTypeSize = sizeof(TT);
        eprintf("REGISTER TYPE: %d %s %d \r\n",GetTypeID(TAG),GetTypeName(TAG).c_str(),sizeof(TT));
      }

      static std::map<int,std::string> TypesID;
      static std::map<int,int> TypesSizes;
      static int MaxTypeSize;
      static int MinTypeSize;
};


template<typename T = void*> 
class TypeRegister
{

public:
      static int     GetTypeID()    { return 0;}
      static size_t  GetTypeCount() { return TypeTagRegister::GetTypeCount();}

      //static std::string GetTypeName()  { return std::string(typeid(T).name()); }

      static std::string GetTypeName()  { return std::string("NO NAME"); }
      static std::string GetTypeName(const int& ID)  { return TypeTagRegister::GetTypeName(ID); }

      static bool isTypeValid()       { return TypeTagRegister::isTypeValid(TypeTagRegister::TYPE_TAG<T>());}
      static bool isTypeValid(const int& ID) { return TypeTagRegister::isTypeValid(ID);}
      static void RegisterType()
      {
    	  TypeTagRegister::RegisterType(TypeTagRegister::TYPE_TAG<T>());
      }

      static int GetTypeSize() { return TypeTagRegister::GetTypeSize(TypeTagRegister::TYPE_TAG<T>()); }
      static int GetMinTypeSize() {return TypeTagRegister::GetMinTypeSize();}
      static int GetMaxTypeSize() {return TypeTagRegister::GetMaxTypeSize();}
};

template<> int TypeRegister<ControlMessage1>::GetTypeID(); 
template<> int TypeRegister<ControlMessage2>::GetTypeID(); 
template<> int TypeRegister<ControlMessage3>::GetTypeID(); 
template<> int TypeRegister<ControlMessage4>::GetTypeID(); 
template<> int TypeRegister<ControlMessage5>::GetTypeID(); 

template<> std::string TypeRegister<ControlMessage1>::GetTypeName();
template<> std::string TypeRegister<ControlMessage2>::GetTypeName();
template<> std::string TypeRegister<ControlMessage3>::GetTypeName();
template<> std::string TypeRegister<ControlMessage4>::GetTypeName();
template<> std::string TypeRegister<ControlMessage5>::GetTypeName();
#endif //TYPE_REGISTER_ENGINE_H
