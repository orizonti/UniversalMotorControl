
#include "RING_BUFFER/TypeRegisterEngine.h"

//std::map<long,long> MessagesMap;

std::map<int,std::string> TypeTagRegister::TypesID;
std::map<int,int> TypeTagRegister::TypesSizes;
int TypeTagRegister::MaxTypeSize = 0;
int TypeTagRegister::MinTypeSize = 0;

template<> int TypeRegister<ControlMessage1>::GetTypeID() { return static_cast<int>(MessageTypes::MessageType1);}
template<> int TypeRegister<ControlMessage2>::GetTypeID() { return static_cast<int>(MessageTypes::MessageType2);}
template<> int TypeRegister<ControlMessage3>::GetTypeID() { return static_cast<int>(MessageTypes::MessageType3);}
template<> int TypeRegister<ControlMessage4>::GetTypeID() { return static_cast<int>(MessageTypes::MessageType4);}
template<> int TypeRegister<ControlMessage5>::GetTypeID() { return static_cast<int>(MessageTypes::MessageType5);}

template<> std::string TypeRegister<ControlMessage1>::GetTypeName()  { return std::string("Message1"); }
template<> std::string TypeRegister<ControlMessage2>::GetTypeName()  { return std::string("Message2"); }
template<> std::string TypeRegister<ControlMessage3>::GetTypeName()  { return std::string("Message3"); }
template<> std::string TypeRegister<ControlMessage4>::GetTypeName()  { return std::string("Message4"); }
template<> std::string TypeRegister<ControlMessage5>::GetTypeName()  { return std::string("Message5"); }
