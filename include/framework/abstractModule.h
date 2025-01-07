#pragma once
#include <thread>
#include<condition_variable>
#include <mutex>
#include <atomic>

namespace DeltaVins {

	class AbstractModule {


	public:
		AbstractModule()
		{
			keepRunning.store(true);
		}

		virtual ~AbstractModule()
		{
			stop();
			delete modulesThread;
		}

		void join()
		{
			if (joinable())_join();
		}
		void start()
		{
			if (!m_bRunning)
				_start();
		}

		virtual void stop()
		{
			this->_stop();
		}
		void detach()
		{
			if (modulesThread != nullptr && m_bRunning)
				if (!m_bDetached)
				{
					_detach();
				}
		}

		bool joinable() const
		{
			if (modulesThread != nullptr)
				return modulesThread->joinable();
			return false;
		}



		void wakeUpMovers()
		{
			std::lock_guard<std::mutex> lk(wakeUpMutex);
			wakeUpConditionVariable.notify_one();
		}

		virtual void runThread()
		{
			while (keepRunning)
			{
				//check for wake up request
				{
					std::unique_lock<std::mutex> ul(wakeUpMutex);
					wakeUpConditionVariable.wait(ul, [this]() { return this->haveThingsTodo() | !this->keepRunning; });
				}
				//do something when wake up
				while (haveThingsTodo() & this->keepRunning)
				{
					doWhatYouNeedToDo();
				}
			}
		}


	protected:
		std::thread* modulesThread = nullptr;
		std::mutex wakeUpMutex;
		std::mutex serialMutex;
		std::condition_variable wakeUpConditionVariable;
		std::condition_variable serialConditionVariable;
		std::atomic_bool keepRunning;
		bool m_bRunning = false;
		bool m_bDetached = false;

		virtual bool haveThingsTodo() = 0;
		virtual void doWhatYouNeedToDo() = 0;

		void waitForThingsToBeDone()
		{

			std::unique_lock<std::mutex> lck(serialMutex);
			serialConditionVariable.wait(lck);

		}

		void tellOthersThingsToBeDone()
		{

			std::unique_lock<std::mutex> ul(serialMutex);
			serialConditionVariable.notify_all();

		}

	private:

		void _stop()
		{
			{
				keepRunning.store(false);
				std::lock_guard<std::mutex> lk(wakeUpMutex);
				wakeUpConditionVariable.notify_one();
			}
			join();
		}

		void _detach()
		{
			modulesThread->detach();
			m_bDetached = true;
		}

		void _start()
		{
			if (m_bRunning)return;
			keepRunning.store(true);

			modulesThread = new std::thread(
				[&]()
				{
					this->runThread();
				});
			m_bRunning = true;
		}

		void _join()
		{
			modulesThread->join();
			m_bRunning = false;
		}

	};
}