#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "x86.h"
#include "proc.h"
#include "spinlock.h"

Ptable ptable;

static struct proc *initproc;

// Reference count for pgdir
extern struct uvmdesc uvmrefcount[NPROC];

int nextpid = 1;
extern void forkret(void);
extern void trapret(void);

static void wakeup1(void *chan);

void pinit(void)
{
  initlock(&ptable.lock, "ptable");
  for (int i = 0; i < NPROC; i++)
    uvmrefcount[i].refcount = -1;
}

// Must be called with interrupts disabled
int cpuid()
{
  return mycpu() - cpus;
}

// Must be called with interrupts disabled to avoid the caller being
// rescheduled between reading lapicid and running through the loop.
struct cpu *
mycpu(void)
{
  int apicid, i;

  if (readeflags() & FL_IF)
    panic("mycpu called with interrupts enabled\n");

  apicid = lapicid();
  // APIC IDs are not guaranteed to be contiguous. Maybe we should have
  // a reverse map, or reserve a register to store &cpus[i].
  for (i = 0; i < ncpu; ++i)
  {
    if (cpus[i].apicid == apicid)
      return &cpus[i];
  }
  panic("unknown apicid\n");
}

// Disable interrupts so that we are not rescheduled
// while reading proc from the cpu structure
struct proc *
myproc(void)
{
  struct cpu *c;
  struct proc *p;
  pushcli();
  c = mycpu();
  p = c->proc;
  popcli();
  return p;
}

// PAGEBREAK: 32
//  Look in the process table for an UNUSED proc.
//  If found, change state to EMBRYO and initialize
//  state required to run in the kernel.
//  Otherwise return 0.
static struct proc *
allocproc(void)
{
  struct proc *p;
  char *sp;

  acquire(&ptable.lock);

  for (p = ptable.proc; p < &ptable.proc[NPROC]; p++)
    if (p->state == UNUSED)
      goto found;

  release(&ptable.lock);
  return 0;

found:
  p->state = EMBRYO;
  p->pid = nextpid++;
  p->nclone = 0;
  p->sleepticks = -1;
  p->chan = 0;

  release(&ptable.lock);

  // Allocate kernel stack.
  if ((p->kstack = kalloc()) == 0)
  {
    p->state = UNUSED;
    return 0;
  }
  sp = p->kstack + KSTACKSIZE;

  // Leave room for trap frame.
  sp -= sizeof *p->tf;
  p->tf = (struct trapframe *)sp;

  // Set up new context to start executing at forkret,
  // which returns to trapret.
  sp -= 4;
  *(uint *)sp = (uint)trapret;

  sp -= sizeof *p->context;
  p->context = (struct context *)sp;
  memset(p->context, 0, sizeof *p->context);
  p->context->eip = (uint)forkret;

  return p;
}

// PAGEBREAK: 32
//  Set up first user process.
void userinit(void)
{
  struct proc *p;
  extern char _binary_initcode_start[], _binary_initcode_size[];

  p = allocproc();

  initproc = p;
  if ((p->pgdir = setupkvm()) == 0)
    panic("userinit: out of memory?");
  inituvm(p->pgdir, _binary_initcode_start, (int)_binary_initcode_size);
  p->sz = PGSIZE;
  memset(p->tf, 0, sizeof(*p->tf));
  p->tf->cs = (SEG_UCODE << 3) | DPL_USER;
  p->tf->ds = (SEG_UDATA << 3) | DPL_USER;
  p->tf->es = p->tf->ds;
  p->tf->ss = p->tf->ds;
  p->tf->eflags = FL_IF;
  p->tf->esp = PGSIZE;
  p->tf->eip = 0; // beginning of initcode.S

  safestrcpy(p->name, "initcode", sizeof(p->name));
  p->cwd = namei("/");

  // this assignment to p->state lets other cores
  // run this process. the acquire forces the above
  // writes to be visible, and the lock is also needed
  // because the assignment might not be atomic.
  acquire(&ptable.lock);

  p->state = RUNNABLE;

  release(&ptable.lock);
}

// Grow current process's memory by n bytes.
// Return 0 on success, -1 on failure.
int growproc(int n)
{
  uint sz;
  struct proc *curproc = myproc();

  sz = curproc->sz;
  if (n > 0)
  {
    if ((sz = allocuvm(curproc->pgdir, sz, sz + n)) == 0)
      return -1;
  }
  else if (n < 0)
  {
    if ((sz = deallocuvm(curproc->pgdir, sz, sz + n)) == 0)
      return -1;
  }
  curproc->sz = sz;
  switchuvm(curproc);
  return 0;
}

// Create a new process copying p as the parent.
// Sets up stack to return as if from system call.
// Caller must set state of returned proc to RUNNABLE.
int fork(void)
{
  int i, pid;
  struct proc *np;
  struct proc *curproc = myproc();

  // Allocate process.
  if ((np = allocproc()) == 0)
  {
    return -1;
  }

  // Copy process state from proc.
  if ((np->pgdir = copyuvm(curproc->pgdir, curproc->sz)) == 0)
  {
    kfree(np->kstack);
    np->kstack = 0;
    np->state = UNUSED;
    return -1;
  }
  np->sz = curproc->sz;
  np->parent = curproc;
  *np->tf = *curproc->tf;

  // Clear %eax so that fork returns 0 in the child.
  np->tf->eax = 0;

  for (i = 0; i < NOFILE; i++)
    if (curproc->ofile[i])
      np->ofile[i] = filedup(curproc->ofile[i]);
  np->cwd = idup(curproc->cwd);

  safestrcpy(np->name, curproc->name, sizeof(curproc->name));

  pid = np->pid;

  acquire(&ptable.lock);

  np->state = RUNNABLE;

  release(&ptable.lock);

  return pid;
}

extern struct uvmdesc *locate_uvmrefcount(pde_t *pgdir);
extern struct uvmdesc *allocate_uvmrefcount(pde_t *pgdir, int refcount);

int clone(void (*fn)(void *), void *stack, void *arg)
{
  int i, pid;
  struct proc *np;
  struct proc *curproc = myproc();

  // Allocate process.
  if ((np = allocproc()) == 0)
  {
    return -1;
  }

  np->pgdir = curproc->pgdir;
  struct uvmdesc *uvmd;
  if ((uvmd = locate_uvmrefcount(curproc->pgdir)) != 0)
  {
    uvmd->refcount += 1;
  }
  else
  {
    uvmd = allocate_uvmrefcount(curproc->pgdir, 2);
  }
  np->chan = 0;
  np->sz = curproc->sz;
  np->parent = curproc;
  *np->tf = *curproc->tf;

  // Clear %eax so that fork returns 0 in the child.
  np->tf->eax = 0;

  uint *ustack = (uint *)stack;
  ustack[-1] = (uint)arg;
  ustack[-2] = 0xffffffffu;

  np->tf->esp = (uint)(ustack - 2);
  np->tf->eip = (uint)fn;

  for (i = 0; i < NOFILE; i++)
    if (curproc->ofile[i])
      np->ofile[i] = filedup(curproc->ofile[i]);
  np->cwd = idup(curproc->cwd);

  // Name the new thread with ID as suffix
  curproc->nclone++;
  safestrcpy(np->name, curproc->name, sizeof(curproc->name));
  i = strlen(np->name);
  int threadnum = curproc->nclone;
  while (threadnum != 0 && i < MAXPROCNAMELEN - 1)
  {
    np->name[i] = '0' + threadnum % 10;
    threadnum /= 10;
    i++;
  }
  np->name[i] = '\0';

  np->nclone++;

  pid = np->pid;

  acquire(&ptable.lock);

  np->state = RUNNABLE;

  release(&ptable.lock);

  return pid;
}

// Exit the current process.  Does not return.
// An exited process remains in the zombie state
// until its parent calls wait() to find out it exited.
void exit(void)
{
  struct proc *curproc = myproc();
  struct proc *p;
  int fd;

  if (curproc == initproc)
    panic("init exiting");

  // Close all open files.
  for (fd = 0; fd < NOFILE; fd++)
  {
    if (curproc->ofile[fd])
    {
      fileclose(curproc->ofile[fd]);
      curproc->ofile[fd] = 0;
    }
  }

  begin_op();
  iput(curproc->cwd);
  end_op();
  curproc->cwd = 0;

  acquire(&ptable.lock);

  // Parent might be sleeping in wait().
  wakeup1(curproc->parent);

  // Pass abandoned children to init.
  for (p = ptable.proc; p < &ptable.proc[NPROC]; p++)
  {
    if (p->parent == curproc)
    {
      p->parent = initproc;
      if (p->state == ZOMBIE)
        wakeup1(initproc);
    }
  }

  // Jump into the scheduler, never to return.
  curproc->state = ZOMBIE;
  sched();
  panic("zombie exit");
}

// Wait for a child process to exit and return its pid.
// Return -1 if this process has no children.
int wait(void)
{
  struct proc *p;
  int havekids, pid;
  struct proc *curproc = myproc();

  acquire(&ptable.lock);
  for (;;)
  {
    // Scan through table looking for exited children.
    havekids = 0;
    for (p = ptable.proc; p < &ptable.proc[NPROC]; p++)
    {
      if (p->parent != curproc)
        continue;
      havekids = 1;
      if (p->state == ZOMBIE)
      {
        // Found one.
        pid = p->pid;
        kfree(p->kstack);
        p->kstack = 0;
        freevm(p->pgdir);
        p->pid = 0;
        p->parent = 0;
        p->name[0] = 0;
        p->killed = 0;
        p->state = UNUSED;
        release(&ptable.lock);
        return pid;
      }
    }

    // No point waiting if we don't have any children.
    if (!havekids || curproc->killed)
    {
      release(&ptable.lock);
      return -1;
    }

    // Wait for children to exit.  (See wakeup1 call in proc_exit.)
    sleep(curproc, &ptable.lock); // DOC: wait-sleep
  }
}

int mholding(mutex *lk)
{
  int r;

  acquire(&lk->lk);
  r = lk->locked && (lk->pid == myproc()->pid);
  release(&lk->lk);
  return r;
}

// PAGEBREAK: 42
//  Per-CPU process scheduler.
//  Each CPU calls scheduler() after setting itself up.
//  Scheduler never returns.  It loops, doing:
//   - choose a process to run
//   - swtch to start running that process
//   - eventually that process transfers control
//       via swtch back to the scheduler.
void scheduler(void)
{
  struct proc *p;
  struct proc *p1;

  struct cpu *c = mycpu();
  c->proc = 0;

  for (;;)
  {
    // Enable interrupts on this processor.
    sti();
    int prevHighPCounter = 0;
    struct proc *prevHighP = 0;  //this is added as sanity check to avoid reusing same p if priority match??need to figure out better way (primarely for test_11)
    struct proc *highP = 0;
    // Loop over process table looking for process to run.
    acquire(&ptable.lock);
    for (p = ptable.proc; p < &ptable.proc[NPROC]; p++)
    {
    //     if (strncmp(p->name, "initcode", sizeof("initcode")) != 0 &&
    //   strncmp(p->name, "init", sizeof("init")) != 0 &&
    //  strncmp(p->name, "sh", sizeof("sh")) != 0 && p->pid>2)   cprintf("starting p = %s, state: %d \n",  p->name, p->state); 
      if (p->state != RUNNABLE)
      {
        continue;
      }
     
      highP = p;
      //int counter = 0;
      for (p1 = ptable.proc; p1 < &ptable.proc[NPROC]; p1++)
      {
        if (p1->state != RUNNABLE)
        {
          continue;
        }
        if (p1->totalM == 0)
        {
          //cprintf("%s->%d: < highP:%s->%d\n",p1->name,p1->priority, highP->name,highP->priority);
          if (p1->priority < highP->priority)
          {

            highP = p1;
          }
        }
        else
        {
          struct proc *px;
          for (px = ptable.proc; px < &ptable.proc[NPROC]; px++)
          {
            if (px->state != RUNNABLE && p1 == px)
            {
              continue;
            }
            // this is for scenario where mutax locks are set and we checkign for priority investion
            // - basically preventing  low priorty  process to hold lock that being waited by high priority process
            for (int i = 0; i < p1->totalM; i++)
            {
              if (p1->m[i].locked && p1->m[i].id > 0 && p1->m[i].id == px->wm && p1->priority > px->priority)
              {
                p1->tempPriority = p1->priority; // back up current priority number
                p1->priority = px->priority;
                if (p1->priority < highP->priority)
                {
                  highP = p;
                }
              }
              else if (p1->priority < highP->priority)
              {
                highP = p1;
              }
            }
          }
        }
      }
      if( prevHighP == highP && !prevHighPCounter)  {
        prevHighPCounter ++;
        continue;
      }
      prevHighPCounter = 0;
      p = highP;
      // Switch to chosen process.  It is the process's job
      // to release ptable.lock and then reacquire it
      // before jumping back to us.
      c->proc = p;
      switchuvm(p);
      p->state = RUNNING;
      swtch(&(c->scheduler), p->context);
      switchkvm();

      // Process is done running for now.
      // It should have changed its p->state before coming back.
      c->proc = 0;
      struct proc *p2;
      // reset priorities to original number
      for (p2 = ptable.proc; p2 < &ptable.proc[NPROC]; p2++)
      {
        if (p2->tempPriority > 0)
        {
          p2->priority = p2->tempPriority;
          p2->tempPriority = 0;
          // cprintf("*** reset prioroty pid: %d\n", p2->pid);
        }
      }
       prevHighP = highP;
    }

    release(&ptable.lock);
  }
}

// Enter scheduler.  Must hold only ptable.lock
// and have changed proc->state. Saves and restores
// intena because intena is a property of this
// kernel thread, not this CPU. It should
// be proc->intena and proc->ncli, but that would
// break in the few places where a lock is held but
// there's no process.
void sched(void)
{
  int intena;
  struct proc *p = myproc();

  if (!holding(&ptable.lock))
    panic("sched ptable.lock");
  if (mycpu()->ncli != 1)
    panic("sched locks");
  if (p->state == RUNNING)
    panic("sched running");
  if (readeflags() & FL_IF)
    panic("sched interruptible");
  intena = mycpu()->intena;
  swtch(&p->context, mycpu()->scheduler);
  mycpu()->intena = intena;
}

// Give up the CPU for one scheduling round.
void yield(void)
{
  acquire(&ptable.lock); // DOC: yieldlock
  myproc()->state = RUNNABLE;
  sched();
  release(&ptable.lock);
}

// A fork child's very first scheduling by scheduler()
// will swtch here.  "Return" to user space.
void forkret(void)
{
  static int first = 1;
  // Still holding ptable.lock from scheduler.
  release(&ptable.lock);

  if (first)
  {
    // Some initialization functions must be run in the context
    // of a regular process (e.g., they call sleep), and thus cannot
    // be run from main().
    first = 0;
    iinit(ROOTDEV);
    initlog(ROOTDEV);
  }

  // Return to "caller", actually trapret (see allocproc).
}

// Atomically release lock and sleep on chan.
// Reacquires lock when awakened.
void sleep(void *chan, struct spinlock *lk)
{
  struct proc *p = myproc();
  if (p == 0)
    panic("sleep");

  if (lk == 0)
    panic("sleep without lk");

  // Must acquire ptable.lock in order to
  // change p->state and then call sched.
  // Once we hold ptable.lock, we can be
  // guaranteed that we won't miss any wakeup
  // (wakeup runs with ptable.lock locked),
  // so it's okay to release lk.
  if (lk != &ptable.lock)
  {                        // DOC: sleeplock0
    acquire(&ptable.lock); // DOC: sleeplock1
    release(lk);
  }
  // Go to sleep.
  p->chan = chan;
  p->state = SLEEPING;
  sched();

  // Tidy up.
  p->chan = 0;

  // Reacquire original lock.
  if (lk != &ptable.lock)
  { // DOC: sleeplock2
    release(&ptable.lock);
    acquire(lk);
  }
}

static void
wakeup1(void *chan)
{
  struct proc *p;

  for (p = ptable.proc; p < &ptable.proc[NPROC]; p++)
  {
    if (p->state == SLEEPING && p->chan == chan)
    {
      p->state = RUNNABLE;
    }
  }
}

// Wake up all processes sleeping on chan.
void wakeup(void *chan)
{
  acquire(&ptable.lock);
  wakeup1(chan);
  release(&ptable.lock);
}

void sleep_proc_notify()
{
  struct proc *p;
  acquire(&ptable.lock);
  for (p = ptable.proc; p < &ptable.proc[NPROC]; p++)
    if (p->state == SLEEPING && p->chan == &ticks)
    {
      if (p->sleepticks == 0)
        p->state = RUNNABLE;
      else
        p->sleepticks -= 1;
    }
  release(&ptable.lock);
}

// Kill the process with the given pid.
// Process won't exit until it returns
// to user space (see trap in trap.c).
int kill(int pid)
{
  struct proc *p;

  acquire(&ptable.lock);
  for (p = ptable.proc; p < &ptable.proc[NPROC]; p++)
  {
    if (p->pid == pid)
    {
      p->killed = 1;
      // Wake process from sleep if necessary.
      if (p->state == SLEEPING)
        p->state = RUNNABLE;
      release(&ptable.lock);
      return 0;
    }
  }
  release(&ptable.lock);
  return -1;
}

// PAGEBREAK: 36
//  Print a process listing to console.  For debugging.
//  Runs when user types ^P on console.
//  No lock to avoid wedging a stuck machine further.
void procdump(void)
{
  static char *states[] = {
      [UNUSED] "unused",
      [EMBRYO] "embryo",
      [SLEEPING] "sleep ",
      [RUNNABLE] "runble",
      [RUNNING] "run   ",
      [ZOMBIE] "zombie"};
  int i;
  struct proc *p;
  char *state;
  uint pc[10];

  for (p = ptable.proc; p < &ptable.proc[NPROC]; p++)
  {
    if (p->state == UNUSED)
      continue;
    if (p->state >= 0 && p->state < NELEM(states) && states[p->state])
      state = states[p->state];
    else
      state = "???";
    cprintf("%d %s %s", p->pid, state, p->name);
    if (p->state == SLEEPING)
    {
      getcallerpcs((uint *)p->context->ebp + 2, pc);
      for (i = 0; i < 10 && pc[i] != 0; i++)
        cprintf(" %p", pc[i]);
    }
    cprintf("\n");
  }
}

void macquire(mutex *lk)
{

  struct proc *p = myproc();
  if (!lk->id)  //in case mutex is not initialized
  {
    lk->id = get_physical_addr_page(p, (uint)lk);
  }
  acquire(&lk->lk);
  while (lk->locked)
  {
    acquire(&ptable.lock);
    p->wm = lk->id;
    release(&ptable.lock);
    sleep(lk, &lk->lk);
  }
  lk->locked = 1;
  acquire(&ptable.lock);
  p->wm = 0;
  p->m[p->totalM] = *lk;
  p->totalM++;
  release(&ptable.lock);
  release(&lk->lk);
}

void mrelease(mutex *lk)
{
  struct proc *p = myproc();
  acquire(&lk->lk);
  // start update proc
  // if (&lk->lk != &ptable.lock)
  // {                        // DOC: sleeplock0
  acquire(&ptable.lock); // DOC: sleeplock1
  //   release(&lk->lk);
  // }
  // go thru maps
  int found = 0;
  for (int i = 0; i < p->totalM; i++)
  {
    if (p->pid == lk->pid)
    {
      found = i;
      break;
    }
  }
  //  if (found == -1) return FAILED; //not found
  // clear & shift em
  memmove(&p->m[found], &p->m[found + 1], sizeof(mutex) * (p->totalM - found - 1));
  p->totalM--;

  // Reacquire original lock.
  // if (&lk->lk != &ptable.lock)
  // { // DOC: sleeplock2
  release(&ptable.lock);
  //   acquire(&lk->lk);
  // }

  lk->locked = 0;
  wakeup(lk);
  release(&lk->lk);
}

int nice(int inc)
{
  struct proc *p = myproc();
  acquire(&ptable.lock);
  if (inc > 19)
    inc = 19;
  else if (inc < -20)
    inc = -20;

  // pushcli();
  p->priority = inc;
  // popcli();
  release(&ptable.lock);
  return 0;
}
