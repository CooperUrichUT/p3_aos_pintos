#include "userprog/syscall.h"
#include <debug.h>
#include <stdio.h>
#include <syscall-nr.h>
#include "devices/input.h"
#include "filesys/file.h"
#include "filesys/filesys.h"
#include "threads/init.h"
#include "threads/interrupt.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"
#include "threads/vaddr.h"
#include "userprog/process.h"
#include "devices/shutdown.h"

static struct lock filesys_lock;

static int get_byte (const uint8_t *uaddr);
static uint32_t get_word (const uint32_t *uaddr);
static bool put_byte (uint8_t *udst, uint8_t byte);
static void validate_read_location (const uint8_t *uaddr, size_t size);
static void validate_write_location (uint8_t *udst, unsigned size);

static void syscall_handler (struct intr_frame *);
struct thread_file *find_file_id (int fd);

static void halt (void);
static void exit (int status);
static tid_t exec (const char *cmd_line);
static int wait (tid_t tid);
static bool create (const char *file, unsigned init_size);
static bool remove (const char *file);
static int open (const char *file);
static int filesize (int fd);
static int read (int fd, void *buffer, unsigned size);
static int write (int fd, void *buffer, unsigned size);
static void seek (int fd, unsigned position);
static unsigned tell (int fd);
static void close (int fd);
int symlink (char *target, char *linkpath);

void syscall_init (void)
{
  lock_init (&filesys_lock);
  intr_register_int (0x30, 3, INTR_ON, syscall_handler, "syscall");
}

static void syscall_handler (struct intr_frame *f UNUSED)
{
  uint32_t *esp = f->esp;
  thread_current ()->esp = esp;

  switch (get_word (esp))
    {
      case SYS_HALT:
        halt ();
        break;
      case SYS_EXIT:
        exit ((int) get_word (esp + 1));
        break;
      case SYS_EXEC:
        f->eax = exec ((const char *) get_word (esp + 1));
        break;
      case SYS_WAIT:
        f->eax = wait ((tid_t) get_word (esp + 1));
        break;
      case SYS_CREATE:
        f->eax = create ((const char *) get_word (esp + 1),
                                 (unsigned) get_word (esp + 2));
        break;
      case SYS_REMOVE:
        f->eax = remove ((const char *) get_word (esp + 1));
        break;
      case SYS_OPEN:
        f->eax = open ((const char *) get_word (esp + 1));
        break;
      case SYS_FILESIZE:
        f->eax = filesize ((int) get_word (esp + 1));
        break;
      case SYS_READ:
        f->eax =
            read ((int) get_word (esp + 1), (void *) get_word (esp + 2),
                          (unsigned) get_word (esp + 3));
        break;
      case SYS_WRITE:
        f->eax = write ((int) get_word (esp + 1),
                                (void *) get_word (esp + 2),
                                (unsigned) get_word (esp + 3));
        break;
      case SYS_SEEK:
        seek ((int) get_word (esp + 1), (unsigned) get_word (esp + 2));
        break;
      case SYS_TELL:
        f->eax = tell ((int) get_word (esp + 1));
        break;
      case SYS_CLOSE:
        close ((int) get_word (esp + 1));
        break;
      case SYS_SYMLINK:
        f->eax = symlink ((char *) get_word (esp + 1), (char *) get_word (esp + 2));
        break;
      default:
        /* Nonexistant system call */
        thread_exit ();
    }
}

static void halt (void) { shutdown_power_off (); }

static void exit (int status)
{
  thread_current ()->st_exit = status;
  thread_exit ();
}

static tid_t exec (const char *cmd_line)
{
  if (!cmd_line || !get_byte (cmd_line))
    return -1;

  tid_t tid = process_execute (cmd_line);

  return tid;
}

static int wait (tid_t tid) { return process_wait (tid); }

static bool create (const char *file, unsigned init_size)
{
  validate_read_location (file, 1);

  lock_acquire (&filesys_lock);
  bool success = filesys_create (file, init_size);
  lock_release (&filesys_lock);
  return success;
}

static bool remove (const char *file)
{
  validate_read_location (file, 1);

  lock_acquire (&filesys_lock);
  bool success = filesys_remove (file);
  lock_release (&filesys_lock);
  return success;
}

static int open (const char *file)
{
  validate_read_location (file, 1);

  lock_acquire (&filesys_lock);
  struct file *f = filesys_open (file);
  if (f == NULL)
    {
      lock_release (&filesys_lock);
      return -1;
    }

  int fd = process_set_file (f);

  lock_release (&filesys_lock);
  return fd;
}

static int filesize (int fd)
{
  lock_acquire (&filesys_lock);
  struct file *file = process_get_file (fd);
  if (file == NULL)
    {
      lock_release (&filesys_lock);
      return -1;
    }

  int size = file_length (file);

  lock_release (&filesys_lock);
  return size;
}

static int read (int fd, void *buffer, unsigned size)
{
  uint8_t *bf = (uint8_t *) buffer;
  validate_read_location (bf, size);
  validate_write_location (bf, size);

  /* Read STDIN (fd = 0) */
  unsigned bytes = 0;
  if (fd == 0)
    {
      uint8_t b;
      while (bytes < size && (b = input_getc ()) != 0)
        {
          *bf++ = b;
          bytes++;
        }
      return (int) bytes;
    }

  lock_acquire (&filesys_lock);
  struct file *file = process_get_file (fd);
  if (file == NULL)
    {
      lock_release (&filesys_lock);
      return -1;
    }

  bytes = file_read (file, buffer, size);

  lock_release (&filesys_lock);
  return (int) bytes;
}

static int write (int fd, void *buffer, unsigned size)
{
  validate_read_location (buffer, size);

  /* Write STDOUT (fd = 1) */
  if (fd == 1)
    {
      putbuf (buffer, size);
      return size;
    }

  lock_acquire (&filesys_lock);
  struct file *file = process_get_file (fd);
  if (file == NULL)
    {
      lock_release (&filesys_lock);
      return -1;
    }

  int bytes = file_write (file, buffer, size);

  lock_release (&filesys_lock);
  return bytes;
}

static void seek (int fd, unsigned position)
{
  lock_acquire (&filesys_lock);
  struct file *file = process_get_file (fd);
  if (file == NULL)
    {
      lock_release (&filesys_lock);
      return;
    }

  file_seek (file, position);

  lock_release (&filesys_lock);
}

static unsigned tell (int fd)
{
  lock_acquire (&filesys_lock);
  struct file *file = process_get_file (fd);
  if (file == NULL)
    {
      lock_release (&filesys_lock);
      return -1;
    }

  unsigned position = file_tell (file);

  lock_release (&filesys_lock);
  return position;
}

static void close (int fd)
{
  struct thread_file *opened_file = find_file_id (fd);
  if (opened_file)
    {
      lock_acquire (&filesys_lock);
      file_close (opened_file->file);
      lock_release (&filesys_lock);
      list_remove (&opened_file->file_elem);

      free (opened_file);
    }
}

int symlink (char *target, char *linkpath)
{
  lock_acquire (&filesys_lock);

  // Check if the target exists
  struct file *target_file = filesys_open (target);
  if (target_file == NULL)
    {
      lock_release (&filesys_lock);
      return -1; // Target does not exist
    }
  // Close the target file as we just needed to check its existence
  file_close (target_file);

  // Check if linkpath already exists
  struct file *link_file = filesys_open (linkpath);
  if (link_file != NULL)
    {
      // Linkpath exists, do not overwrite, close and exit
      file_close (link_file);
      lock_release (&filesys_lock);
      return -1;
    }

  bool success = filesys_symlink (target, linkpath);
  lock_release (&filesys_lock);

  return success ? 0 : -1;
}

static int get_byte (const uint8_t *uaddr)
{
  if (!is_user_vaddr (uaddr))
    return -1;
  int result;
  asm ("movl $1f, %0; movzbl %1, %0; 1:" : "=&a"(result) : "m"(*uaddr));
  return result;
}

static uint32_t get_word (const uint32_t *uaddr)
{
  uint32_t word = 0;
  for (int offset = 0; offset < sizeof (uint32_t); ++offset)
    {
      int byte_value = get_byte ((const uint8_t *) uaddr + offset);
      if (byte_value == -1)
        {
          exit (-1);
        }
      *((uint8_t *) &word + offset) = (uint8_t) byte_value;
    }
  return word;
}

static bool put_byte (uint8_t *udst, uint8_t byte)
{
  if (!is_user_vaddr (udst))
    return false;
  int error_code;
  asm ("movl $1f, %0; movb %b2, %1; 1:"
       : "=&a"(error_code), "=m"(*udst)
       : "r"(byte));
  return error_code != -1;
}

static void validate_read_location (const uint8_t *uaddr, size_t size)
{
  uint8_t *ptr;
  for (ptr = pg_round_down (uaddr); ptr < uaddr + size; ptr += PGSIZE)
    {
      if (get_byte (ptr) == -1)
        {
          exit (-1);
        }
    }
}

static void validate_write_location (uint8_t *udst, unsigned size)
{
  uint8_t *ptr;
  for (ptr = pg_round_down (udst); ptr < udst + size; ptr += PGSIZE)
    {
      if (!put_byte (ptr, get_byte (ptr)))
        {
          exit (-1);
        }
    }
}

struct thread_file *find_file_id (int file_id)
{
  struct list_elem *e;
  struct thread_file *thread_file_temp = NULL;
  struct list *files = &thread_current ()->files;
  for (e = list_begin (files); e != list_end (files); e = list_next (e))
    {
      thread_file_temp = list_entry (e, struct thread_file, file_elem);
      if (file_id == thread_file_temp->fd)
        return thread_file_temp;
    }
  return false;
}
