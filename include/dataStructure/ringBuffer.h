#pragma once


#include <cassert>
#include <utils/log.h>
#include <typeinfo>
#include <cstring>

namespace DeltaVins {


	template<typename T,int N>
	struct CircularBuffer
	{
	public:
		using BufferIndex = int;
		CircularBuffer():_BUFSIZE(1<<N),_END(_BUFSIZE-1){
			_buf = new T[_BUFSIZE];
		}

		~CircularBuffer()
		{
			if (_buf)
				delete[] _buf;
		}

		bool empty() const{

			return m_head == m_tail;

		};

		bool full() const{

			return (m_head + 1 & _END) == m_tail;
		
		};

		BufferIndex getDeltaIndex(BufferIndex index, BufferIndex delta)const {
			return index + _BUFSIZE + delta & _END;

		}

		T& getHeadNode() {
			return _buf[m_head];
		};

		// 
		void pushIndex() {
		    if(getDeltaIndex(m_head,1)==m_tail){
                m_tail = m_tail + 1 & _END;
		    }
			m_head = m_head + 1 & _END;
		}

		//
		void popIndex() {

			m_tail = m_tail + 1 & _END;

		}

		enum BinarySearchResultType
		{
			Left = 0,
			Right = 1,
			Exact = -1
		};


		template<typename Key>
        BufferIndex binarySearch(const Key &key, BinarySearchResultType leftOrRight) const {
			//If buffer is empty,return false
			if (empty())
				return -3;
			int head_M1 = getDeltaIndex(m_head, -1);
			int tail_P1 = getDeltaIndex(m_tail, 5);

			if (_buf[head_M1]<key) {

			    //if constexpr (typeid(key) == typeid(long long int))
                    //LOGW("Image is Faster than IMU,tail:%d %d head:%d %dkey:%lld,tail_P1:%lld,head_M1:%lld",m_tail,tail_P1,m_head,head_M1,key,_buf[tail_P1].timestamp,_buf[head_M1].timestamp)
                return -1;
            }
			if(_buf[tail_P1]>key){
                //if constexpr (typeid(key) == typeid(int))
					//LOGE("Image Idx:%d tailIdx:%d",key,_buf[tail_P1].idx);
				    return -3;
			}

			int left;
			int right;
			//set binary search left and right side
			if (head_M1 > tail_P1)
			{
				left = tail_P1;
				right = head_M1;
			}
			else
			{
				if (_buf[0] <= key)
				{
					left = 0;
					right = head_M1;
				}
				else
				{
					if (_buf[_END] <= key)
					{
						left = _END;
						right = 0;
					}
					else
					{
						left = tail_P1;
						right = _END;
					}
				}
			}
			//do binary search
			while (left + 1 < right)
			{
				int mid = (left + right) / 2;
				if (_buf[mid] == key)
					return mid;
				if (_buf[mid] <= key)
					left = mid;
				else
					right = mid;
			}
            if(leftOrRight == 0)
                return left;
            else if(leftOrRight == 1)
                return right;
            else if(leftOrRight == -1)
                return -2;
            else
                assert(0&&"Wrong LeftOrRight");
		}

	protected:
		const BufferIndex _BUFSIZE;
		const BufferIndex _END;
		T* _buf = nullptr;
		//std::atomic<BufferIndex> m_head, m_tail;
		BufferIndex m_head, m_tail;

	};

}