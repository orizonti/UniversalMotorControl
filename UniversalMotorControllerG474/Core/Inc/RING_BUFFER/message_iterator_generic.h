#ifndef MESSAGE_ITERATORGENERIC_H
#define MESSAGE_ITERATORGENERIC_H
#include <cstdint>
#include <cstring>
#include "message_struct_generic.h"



//template<typename H> class MessageIteratorGenericBase;
enum class IteratorMode {Chunked = 0, ChunkedFixed = 1, ChunkedContinous = 2, Continous = 3};

template<typename H>
class MessageIteratorGenericBase
{
public:
    MessageIteratorGenericBase();
    MessageIteratorGenericBase(uint8_t* STORAGE, std::size_t ChunkSize, std::size_t NumberChunks);
	struct MessagesRange
	{
		uint8_t* EndMessageBuffer;
		uint8_t* LastMessage;
	};

public:
	virtual void LoadData(uint8_t* DataSourceBuffer, uint16_t BytesCountReceived) = 0;
	MessageIteratorGenericBase<H> operator++(int);

	MessageGeneric<void*,H>* GetMessagePtr();
	MessageGeneric<void*,H>& operator*();

	bool IsHeaderValid();

	void ResetIterator();

	int DataAvailable();

	int MessageSize();
	int MessageSize(uint8_t* PtrMessage);

	H&  GetHeader();
	H&  GetHeader(uint8_t* PtrHeader);

	int  StepsTo(const MessageIteratorGenericBase<H>& It) { return It.MessageNumber - MessageNumber;}; 

	void operator=(const MessageIteratorGenericBase& Message);
	bool operator==(MessageIteratorGenericBase& Message);
	bool operator!=(MessageIteratorGenericBase& Message);

	MessagesRange RangeLimits;
	uint32_t MessageNumber = 0;

    void PrintIterator();
	protected:
	uint8_t* PtrMessageBegin;
	uint8_t* PtrMessageBeginPrevious;
	uint8_t* PtrDataEnd = 0;

	uint8_t* PtrBufferBegin;
	uint8_t* PtrBufferEnd;

	uint8_t  BufferChunkSize = 100;
	uint8_t  MessageDataSize = 25;
	
	void SwitchToNext();
};



template<typename H>
MessageIteratorGenericBase<H>::MessageIteratorGenericBase()
{
	PtrBufferBegin = 0; PtrBufferEnd = 0; PtrMessageBegin = 0; PtrDataEnd = 0;
}

template<typename H>
MessageIteratorGenericBase<H>::MessageIteratorGenericBase(uint8_t* STORAGE, std::size_t ChunkSize, std::size_t NumberChunks)
{
	BufferChunkSize = ChunkSize;
	PtrBufferBegin = STORAGE; PtrBufferEnd = STORAGE + NumberChunks*ChunkSize;
	PtrMessageBegin = PtrBufferBegin; PtrDataEnd = PtrMessageBegin;

	RangeLimits.EndMessageBuffer = PtrBufferEnd;
	RangeLimits.LastMessage = PtrMessageBegin; 
}






template<typename H>
void MessageIteratorGenericBase<H>::operator=(const MessageIteratorGenericBase<H>& Message)
{
	PtrMessageBegin = Message.PtrMessageBegin; 
	     PtrDataEnd = Message.PtrDataEnd;
	 PtrBufferBegin = Message.PtrBufferBegin;
 	   PtrBufferEnd = Message.PtrBufferEnd;
	   BufferChunkSize = Message.BufferChunkSize;
	   RangeLimits.EndMessageBuffer = Message.RangeLimits.EndMessageBuffer;
	   RangeLimits.LastMessage      = Message.RangeLimits.LastMessage;
}


//template<typename H> void MessageIteratorGenericBase<H>::operator<<(DataSourceClass& DataSource) { }

template<typename H> bool MessageIteratorGenericBase<H>::operator!=(MessageIteratorGenericBase<H>& Message) { return !(Message == *this); }
template<typename H> bool MessageIteratorGenericBase<H>::operator==(MessageIteratorGenericBase<H>& Message) { return PtrMessageBegin == Message.PtrMessageBegin; }

template<typename H> bool MessageIteratorGenericBase<H>::IsHeaderValid() { return GetHeader().isValid(); }
template<typename H>   H& MessageIteratorGenericBase<H>::GetHeader()     { return *reinterpret_cast<H*>(PtrMessageBegin); }
template<typename H>   H& MessageIteratorGenericBase<H>::GetHeader(uint8_t* PtrHeader) { return *reinterpret_cast<H*>(PtrHeader); }

template<typename H> int MessageIteratorGenericBase<H>::MessageSize(uint8_t* PtrMessage)
{
	auto& Header = GetHeader(PtrMessage); 
	if(!Header.isValid()) return 0; 
						  return Header.DATA_SIZE + sizeof(H);
}
template<typename H> int MessageIteratorGenericBase<H>::MessageSize()      
{ 
	auto& Header = GetHeader(); 
	if(!Header.isValid()) return 0; 
						  return Header.DATA_SIZE + sizeof(H);
};

template<typename H> int MessageIteratorGenericBase<H>::DataAvailable()   { return PtrDataEnd - PtrMessageBegin; }

template<typename H> void MessageIteratorGenericBase<H>::ResetIterator()  
{ PtrMessageBegin = PtrBufferBegin; PtrDataEnd = PtrBufferBegin; }


template<typename H> 
MessageGeneric<void*,H>& MessageIteratorGenericBase<H>::operator*() 
{ 
	return *reinterpret_cast<MessageGeneric<void*,H>* >(PtrMessageBegin); 
}
template<typename H> 
MessageGeneric<void*,H>* MessageIteratorGenericBase<H>::GetMessagePtr() { return reinterpret_cast<MessageGeneric<void*,H>* >(PtrMessageBegin); }


template<typename H>
void MessageIteratorGenericBase<H>::PrintIterator() { }

template<typename H>
void MessageIteratorGenericBase<H>::SwitchToNext()
{
 if(this->PtrMessageBegin == this->RangeLimits.LastMessage) return;

    this->MessageNumber++;   this->PtrMessageBegin += this->BufferChunkSize; 

 if(this->PtrMessageBegin == this->RangeLimits.EndMessageBuffer) 
    this->PtrMessageBegin =  this->PtrBufferBegin; 
}

template<typename H>
MessageIteratorGenericBase<H> MessageIteratorGenericBase<H>::operator++(int) { SwitchToNext(); return* this; }


//=============================================================

template<typename H, IteratorMode Mode = IteratorMode::ChunkedFixed>
class MessageIteratorGeneric : public MessageIteratorGenericBase<H> 
{
public:
    MessageIteratorGeneric(): MessageIteratorGenericBase<H>() {};
    MessageIteratorGeneric(uint8_t* STORAGE, std::size_t ChunkSize, std::size_t NumberChunks) :
	MessageIteratorGenericBase<H>(STORAGE, ChunkSize, NumberChunks) { };

public:
	void LoadData(uint8_t* DataSourceBuffer, uint16_t BytesCountReceived);
    MessageIteratorGeneric<H,Mode> operator++(int) { this->SwitchToNext(); return* this; };
};


    //===============================================================================================
	template<typename H, IteratorMode Mode>
	void MessageIteratorGeneric<H,Mode>::LoadData(uint8_t* DataSourceBuffer, uint16_t BytesCountReceived)
	{
	if(BytesCountReceived < this->BufferChunkSize) return;

	std::memcpy(this->PtrMessageBegin,DataSourceBuffer, this->BufferChunkSize); this->MessageNumber++;
				this->PtrMessageBegin += this->BufferChunkSize; 

								this->RangeLimits.LastMessage = this->PtrMessageBegin; 
	if(this->PtrMessageBegin == this->RangeLimits.EndMessageBuffer) //RESET TO BEGIN
		this->PtrMessageBegin  = this->PtrBufferBegin; 

	LoadData(DataSourceBuffer + this->BufferChunkSize, BytesCountReceived - this->BufferChunkSize);
	}
    //===============================================================================================

template<typename H>
class MessageIteratorGeneric<H,IteratorMode::Chunked> : public MessageIteratorGenericBase<H> 
{
public:
    MessageIteratorGeneric(): MessageIteratorGenericBase<H>() {};
    MessageIteratorGeneric(uint8_t* STORAGE, std::size_t ChunkSize, std::size_t NumberChunks) :
	MessageIteratorGenericBase<H>(STORAGE, ChunkSize, NumberChunks) { };

public:
	void LoadData(uint8_t* DataSourceBuffer, uint16_t BytesCountReceived);

    MessageIteratorGeneric<H,IteratorMode::Chunked> operator++(int) { this->SwitchToNext(); return* this; };
};

    //===============================================================================================
	template<typename H>
	void MessageIteratorGeneric<H,IteratorMode::Chunked>::LoadData(uint8_t* DataSourceBuffer, uint16_t BytesCountReceived)
	{
	if(!this->GetHeader(DataSourceBuffer).isValid()) return;

														this->MessageDataSize = this->MessageSize(DataSourceBuffer);
	std::memcpy(this->PtrMessageBegin,DataSourceBuffer, this->MessageDataSize); this->MessageNumber++;
				this->PtrMessageBegin += this->BufferChunkSize; 

								this->RangeLimits.LastMessage = this->PtrMessageBegin; 
	if(this->PtrMessageBegin == this->RangeLimits.EndMessageBuffer) //RESET TO BEGIN
		this->PtrMessageBegin  = this->PtrBufferBegin; 
	}
    //===============================================================================================

template<typename H>
class MessageIteratorGeneric<H,IteratorMode::ChunkedContinous> : public MessageIteratorGenericBase<H> 
{
public:
    MessageIteratorGeneric(): MessageIteratorGenericBase<H>() {};
    MessageIteratorGeneric(uint8_t* STORAGE, std::size_t ChunkSize, std::size_t NumberChunks) :
	MessageIteratorGenericBase<H>(STORAGE, ChunkSize, NumberChunks)
	{
	};

public:
	void LoadData(uint8_t* DataSourceBuffer, uint16_t BytesCountReceived);

    MessageIteratorGeneric<H,IteratorMode::ChunkedContinous> operator++(int) { this->SwitchToNext(); return* this; };
};


    //===============================================================================================
	template<typename H>
	void MessageIteratorGeneric<H,IteratorMode::ChunkedContinous>::LoadData(uint8_t* DataSourceBuffer, uint16_t BytesCountReceived)
	{
	auto& PtrMessage = this->PtrMessageBegin;
	int MessageBytesAvailable = this->DataAvailable();
	int BytesToLoad = 0;

	if(BytesCountReceived <= 0) return;
	if(BytesCountReceived <= sizeof(H))  
	{ 
		std::memcpy(this->PtrDataEnd,DataSourceBuffer, BytesCountReceived); 
					this->PtrDataEnd += BytesCountReceived; return;
	}

	if(MessageBytesAvailable < sizeof(H) && MessageBytesAvailable != 0) 
	{
		BytesToLoad = (sizeof(H) - MessageBytesAvailable); 
		std::memcpy(this->PtrDataEnd,DataSourceBuffer, BytesToLoad);
					this->PtrDataEnd += BytesToLoad; 

					DataSourceBuffer += BytesToLoad;
					BytesCountReceived -= BytesToLoad;
					MessageBytesAvailable += BytesToLoad;
	}


	BytesToLoad = 0;
	if(this->GetHeader(DataSourceBuffer).isValid()) BytesToLoad = this->MessageSize(DataSourceBuffer) - MessageBytesAvailable; 
	if(this->GetHeader(PtrMessage ).isValid())      BytesToLoad = this->MessageSize(PtrMessage ) - MessageBytesAvailable; 
	if(BytesToLoad == 0) 
	{
		this->MessageNumber++;                                
									this->PtrMessageBegin += this->BufferChunkSize;
	this->RangeLimits.LastMessage = this->PtrMessageBegin;
				 this->PtrDataEnd = this->PtrMessageBegin; return;
	}

	if(BytesToLoad > BytesCountReceived ) 
	{ 
		std::memcpy(this->PtrDataEnd,DataSourceBuffer, BytesCountReceived); 
					this->PtrDataEnd += BytesCountReceived; 
					return; //WE DIDNT GET FULL MESSAGE HERE
	}

	if(BytesToLoad <= BytesCountReceived)
	{
		std::memcpy(this->PtrDataEnd,DataSourceBuffer, BytesToLoad); 

		this->MessageNumber++;                                
									this->PtrMessageBegin += this->BufferChunkSize;
	this->RangeLimits.LastMessage = this->PtrMessageBegin;
				 this->PtrDataEnd = this->PtrMessageBegin;
	}

	if(this->PtrMessageBegin == this->RangeLimits.EndMessageBuffer) //RESET TO BEGIN
	{
		this->PtrMessageBegin  = this->PtrBufferBegin; 
	}

	LoadData(DataSourceBuffer + BytesToLoad, BytesCountReceived - BytesToLoad);
	}
    //===============================================================================================

template<typename H>
class MessageIteratorGeneric<H,IteratorMode::Continous> : public MessageIteratorGenericBase<H> 
{
public:
    MessageIteratorGeneric(): MessageIteratorGenericBase<H>() {};
    MessageIteratorGeneric(uint8_t* STORAGE, std::size_t ChunkSize, std::size_t NumberChunks) :
	MessageIteratorGenericBase<H>(STORAGE, ChunkSize, NumberChunks)
	{
	};

public:
	void LoadData(uint8_t* DataSourceBuffer, uint16_t BytesCountReceived);

    MessageIteratorGeneric<H,IteratorMode::Continous> operator++(int); 
protected:

    void MoveDataToBegin();
	bool IsMemoryAtEnd();
	int MemoryAvailable();
	int DataAvailable();
	uint8_t* PtrMessageBeginPrevious; 
};

    template<typename H> bool MessageIteratorGeneric<H,IteratorMode::Continous>::IsMemoryAtEnd() 
	{ return (this->PtrBufferEnd - this->PtrMessageBegin) < 2*this->BufferChunkSize; }

    template<typename H> int MessageIteratorGeneric<H,IteratorMode::Continous>::MemoryAvailable()
	{ return (this->PtrBufferEnd - this->PtrMessageBegin) - 2*this->BufferChunkSize; }

    template<typename H> int MessageIteratorGeneric<H,IteratorMode::Continous>::DataAvailable()  
	{ return std::abs(this->PtrDataEnd - this->PtrMessageBegin); }

    template<typename H>
    MessageIteratorGeneric<H,IteratorMode::Continous> MessageIteratorGeneric<H, IteratorMode::Continous>::operator++(int) 
    {
     if(this->PtrMessageBegin == this->RangeLimits.LastMessage) return *this;
    
     auto& Header = this->GetHeader(); if(!Header.isValid()) { this->ResetIterator(); return *this;};
    
        this->MessageNumber++;
        this->PtrMessageBegin += sizeof(H) + Header.DATA_SIZE; 
     if(this->PtrMessageBegin == this->RangeLimits.EndMessageBuffer) 
        this->PtrMessageBegin =  this->PtrBufferBegin; 
    
     return* this;
    }
    
    template<typename H>
    void MessageIteratorGeneric<H,IteratorMode::Continous>::MoveDataToBegin()
    {
    	                                               uint8_t DataRemain = this->DataAvailable();
    	            this->PtrDataEnd = this->PtrBufferBegin  + DataRemain;
    	std::memcpy(this->PtrBufferBegin,this->PtrMessageBegin,DataRemain); 
    	            this->PtrMessageBegin = this->PtrBufferBegin;
    }

    //===============================================================================================
	template<typename H>
	void MessageIteratorGeneric<H,IteratorMode::Continous>::LoadData(uint8_t* DataSourceBuffer, uint16_t BytesCountReceived)
	{
	std::memcpy(this->PtrDataEnd,DataSourceBuffer,BytesCountReceived); this->PtrDataEnd += BytesCountReceived;

		while(DataAvailable() >=this->MessageSize())
			{
				if(!this->IsHeaderValid()) { this->ResetIterator(); return; }

				                                                    PtrMessageBeginPrevious = this->PtrMessageBegin; 
				this->PtrMessageBegin += this->MessageSize(); this->RangeLimits.LastMessage = this->PtrMessageBegin;
				this->MessageNumber++;

				if(IsMemoryAtEnd()) 
				{
					this->RangeLimits.EndMessageBuffer = this->PtrMessageBegin;
					MoveDataToBegin();
				} 
				if(DataAvailable() < sizeof(H)) return;

			}
	}
//====================================================================================================

#endif //MESSAGE_ITERATORGENERIC_H

