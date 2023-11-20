// Buffer cache.
//
// The buffer cache is a linked list of buf structures holding
// cached copies of disk block contents.  Caching disk blocks
// in memory reduces the number of disk reads and also provides
// a synchronization point for disk blocks used by multiple processes.
//
// Interface:
// * To get a buffer for a particular disk block, call bread.
// * After changing buffer data, call bwrite to write it to disk.
// * When done with the buffer, call brelse.
// * Do not use the buffer after calling brelse.
// * Only one process at a time can use a buffer,
//     so do not keep them longer than necessary.


#include "types.h"
#include "param.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "riscv.h"
#include "defs.h"
#include "fs.h"
#include "buf.h"

#define NBUCKET 13
#define BUCKET_HASH(dev, blockno) ((dev + blockno) % NBUCKET)

struct bucket {
	struct spinlock lock;
	struct buf head;  // sentinel node, helpful in buffer eviction.
};

struct {
  struct spinlock lock;
  struct buf buf[NBUF];
	struct bucket buckets[NBUCKET];
} bcache;

void
binit(void)
{
  initlock(&bcache.lock, "bcache");

	for(int i = 0; i < NBUCKET; i++)
		initlock(&bcache.buckets[i].lock, "bcache.bucket");

	// create buffer hashtable, put all bufs to buckets[0]
  for(struct buf *b = bcache.buf; b < bcache.buf+NBUF; b++){
    initsleeplock(&b->lock, "buffer");
    b->next = bcache.buckets[0].head.next;
		bcache.buckets[0].head.next = b;
  }
}

// Look through buffer cache for block on device dev.
// If not found, allocate a buffer.
// In either case, return locked buffer.
static struct buf*
bget(uint dev, uint blockno)
{
  struct buf *b;
	int nbuc = BUCKET_HASH(dev, blockno);

	// Is the block already cached?
	acquire(&bcache.buckets[nbuc].lock);
  for(b = bcache.buckets[nbuc].head.next; b != 0; b = b->next){
    if(b->dev == dev && b->blockno == blockno){
      b->refcnt++;
      release(&bcache.buckets[nbuc].lock);
      acquiresleep(&b->lock);
      return b;
    }
  }
	release(&bcache.buckets[nbuc].lock); // to prevent deadlock

  // Not cached.
	acquire(&bcache.lock);

	// Is already cached? (again)
	// For concurrent "acquire(&bcache.lock)" when searching for the same buf.
	for(b = bcache.buckets[nbuc].head.next; b != 0; b = b->next){
		if(b->dev == dev && b->blockno == blockno){
			acquire(&bcache.buckets[nbuc].lock);
			b->refcnt++;
			release(&bcache.buckets[nbuc].lock);
			release(&bcache.lock);
			acquiresleep(&b->lock);
			return b;
		}
	}

	// Still not cached.

	// For LRU buf eviction.
	uint min_ticks = 0xffffffff;
	int lru_found_thisbuc = 0;
	int lru_nbuc = 0;
	struct buf *lru_buf = 0;

	// Traverse to find LRU buffer (0 indicates no free buffer)
	for(int i = 0; i < NBUCKET; i++){

		acquire(&bcache.buckets[i].lock);

		for(b = bcache.buckets[i].head.next; b != 0; b = b->next){
			if(b->refcnt == 0 && b->timestamp < min_ticks){
				// release lock of bucket where the prev lru_buf located
				if(lru_buf != 0 && lru_found_thisbuc == 0)
					release(&bcache.buckets[lru_nbuc].lock);

				lru_found_thisbuc = 1;
				lru_nbuc = i;
				lru_buf = b;
			}
		}

		// hold lock if LRU buffer is found in this bucket
		if(lru_found_thisbuc){
			lru_found_thisbuc = 0;
		} else{
			release(&bcache.buckets[i].lock);
		}
	}

	// Evict, modify and insert the LRU buffer.

	if(lru_buf){
		// Evict. Use sentinel node and release bucket lock where LRU buffer located.
		for(b = &bcache.buckets[lru_nbuc].head; b->next != 0; b = b->next){
			if(b->next == lru_buf){
				b->next = lru_buf->next;
				release(&bcache.buckets[lru_nbuc].lock);
				// release bcache.lock as soon as possible
				acquire(&bcache.buckets[nbuc].lock);
				release(&bcache.lock);
				break;
			}
		}
		
		// Modify.
		lru_buf->dev = dev;
		lru_buf->blockno = blockno;
		lru_buf->valid = 0;
		lru_buf->refcnt = 1;

		// Insert.
		lru_buf->next = bcache.buckets[nbuc].head.next;
		bcache.buckets[nbuc].head.next = lru_buf;
		release(&bcache.buckets[nbuc].lock);

		acquiresleep(&lru_buf->lock);

		return lru_buf;
	}

	panic("bget: no buffers");
}

// Return a locked buf with the contents of the indicated block.
struct buf*
bread(uint dev, uint blockno)
{
  struct buf *b;

  b = bget(dev, blockno);
  if(!b->valid) {
    virtio_disk_rw(b, 0);
    b->valid = 1;
  }
  return b;
}

// Write b's contents to disk.  Must be locked.
void
bwrite(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("bwrite");
  virtio_disk_rw(b, 1);
}

// Release a locked buffer.
// Move to the head of the most-recently-used list.
void
brelse(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("brelse");

  releasesleep(&b->lock);

	int nbuc = BUCKET_HASH(b->dev, b->blockno);

	// Since i++/i-- is not atomic!
  acquire(&bcache.buckets[nbuc].lock);
  b->refcnt--;
	// update timestamp when the buff is free
	if(b->refcnt == 0)
		b->timestamp = ticks;
  release(&bcache.buckets[nbuc].lock);
}

void
bpin(struct buf *b) {
	int nbuc = BUCKET_HASH(b->dev, b->blockno);

  acquire(&bcache.buckets[nbuc].lock);
  b->refcnt++;
  release(&bcache.buckets[nbuc].lock);
}

void
bunpin(struct buf *b) {
	int nbuc = BUCKET_HASH(b->dev, b->blockno);

  acquire(&bcache.buckets[nbuc].lock);
  b->refcnt--;
  release(&bcache.buckets[nbuc].lock);
}

