#include <adtf_utils.h>

class MutexLocker
{
public:
	MutexLocker(adtf_util::cMutex *m) {
		this->m = m;
		m->Enter();
	}
	~MutexLocker() {
		m->Leave();
	}

private:
	adtf_util::cMutex *m;
};
