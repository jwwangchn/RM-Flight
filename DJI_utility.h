#ifndef __DJI_UTILITY_H__
#define __DJI_UTILITY_H__
#include <stdio.h>

#ifdef WIN32

#include <Windows.h>
#include <WinBase.h>

class DJI_lock
{
public:
	DJI_lock();
	~DJI_lock();
	void         enter();
	void         leave();
private:
	CRITICAL_SECTION  m_critical_section;
};

class DJI_event
{
public:
	DJI_event();
	~DJI_event();
	int         set_event();
	int         wait_event();
private:
	HANDLE      m_pipe_read;
	HANDLE      m_pipe_write;
};

#else

#include <pthread.h>
#include <semaphore.h>

class DJI_lock
{
public:
	DJI_lock();
	~DJI_lock();
	void         enter();
	void         leave();
private:
	pthread_mutex_t m_lock;
};

class DJI_event
{
public:
	DJI_event();
	~DJI_event();
	int         set_event();
	int         wait_event();
private:
	sem_t		m_sem;
};

#endif



#endif




#ifdef WIN32

DJI_lock::DJI_lock()
{
	InitializeCriticalSection( &m_critical_section );
}

DJI_lock::~DJI_lock()
{
}

void DJI_lock::enter()
{
	EnterCriticalSection( &m_critical_section );
}

void DJI_lock::leave()
{
	LeaveCriticalSection( &m_critical_section );
}


DJI_event::DJI_event()
{
	SECURITY_ATTRIBUTES   sa;
	sa.nLength = sizeof(sa);
	sa.lpSecurityDescriptor = NULL;
	sa.bInheritHandle = TRUE;
	CreatePipe( &m_pipe_read, &m_pipe_write, &sa, 0 );
}

DJI_event::~DJI_event()
{
	CloseHandle( m_pipe_read );
	CloseHandle( m_pipe_write );
}

int DJI_event::set_event()
{
	char buffer[1] = {0};
	int count = 0;
	int ret = WriteFile( m_pipe_write, (LPWORD)buffer, 1, (LPDWORD)&count, NULL );
	return ret;
}

int DJI_event::wait_event()
{
	char buffer[1] = {0};
	int count = 0;
	int ret = ReadFile( m_pipe_read, (LPWORD)buffer, 1, (LPDWORD)&count, NULL );
	return ret;
}

#else

DJI_lock::DJI_lock()
{
	pthread_mutex_init( &m_lock, NULL );
}

DJI_lock::~DJI_lock()
{
}

void DJI_lock::enter()
{
	pthread_mutex_lock( &m_lock );
}

void DJI_lock::leave()
{
	pthread_mutex_unlock( &m_lock );
}

DJI_event::DJI_event()
{
	sem_init( &m_sem, 0, 0 );
}

DJI_event::~DJI_event()
{
}

int DJI_event::set_event()
{
	int ret = sem_post( &m_sem );
	return ret;
}

int DJI_event::wait_event()
{
	int ret = sem_wait( &m_sem );
	return ret;
}

#endif
