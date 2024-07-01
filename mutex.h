typedef struct __mutex
{
    uint locked;        // Is the lock held?
    struct spinlock lk; // spinlock protecting this this lock
    int pid;
    int id;
} mutex;

void            initlock(struct spinlock*, char*);
void minit(mutex *);
void macquire(mutex *);
void mrelease(mutex *);
int nice(int);