import pytest
import threading
import time
from depthai import MessageQueue, Buffer


def test_basic_operations():
    queue = MessageQueue("test", maxSize=10, blocking=True)

    # Test send and get
    msg1 = Buffer()
    queue.send(msg1)
    received_msg1 = queue.get()
    assert received_msg1 == msg1

    # Test tryGet
    msg2 = Buffer()
    queue.send(msg2)
    received_msg2 = queue.tryGet()
    assert received_msg2 == msg2

    # Test empty tryGet
    null_msg = queue.tryGet()
    assert null_msg is None


def test_blocking_behavior():
    queue = MessageQueue("test", maxSize=1, blocking=True)

    msg1 = Buffer()
    msg2 = Buffer()

    # Test blocking send
    queue.send(msg1)

    def send_thread():
        queue.send(msg2)

    thread = threading.Thread(target=send_thread)
    thread.start()
    time.sleep(0.1)
    assert queue.get() == msg1
    thread.join()
    assert queue.get() == msg2


def test_non_blocking_behavior():
    queue = MessageQueue("test", maxSize=1, blocking=False)

    msg1 = Buffer()
    msg2 = Buffer()

    # Test non-blocking send
    queue.send(msg1)
    queue.send(msg2)
    assert queue.get() == msg2


def test_multiple_producers_and_consumers():
    queue = MessageQueue("test", maxSize=100, blocking=True)
    num_messages = 10000
    num_producers = 10
    num_consumers = 10
    num_messages_per_producer = num_messages // num_producers
    num_messages_per_consumer = num_messages // num_consumers

    sent_count = 0
    received_count = 0

    def producer():
        nonlocal sent_count
        for _ in range(num_messages_per_producer):
            msg = Buffer()
            queue.send(msg)
            sent_count += 1

    def consumer():
        nonlocal received_count
        for _ in range(num_messages_per_consumer):
            msg = queue.get()
            received_count += 1

    producers = [threading.Thread(target=producer) for _ in range(num_producers)]
    consumers = [threading.Thread(target=consumer) for _ in range(num_consumers)]

    for p in producers:
        p.start()
    for c in consumers:
        c.start()

    for p in producers:
        p.join()
    for c in consumers:
        c.join()

    assert sent_count == num_messages
    assert received_count == num_messages


def test_timeout():
    queue = MessageQueue("test", maxSize=10, blocking=True)

    # Test get with timeout
    msg = queue.get(timeout=0.1)
    assert msg is None

    # Test send with timeout
    large_msg = Buffer()
    send_timed_out = not queue.send(large_msg, timeout=0.1)
    assert not send_timed_out


def test_callbacks():
    queue = MessageQueue("test", maxSize=10, blocking=True)
    callback_count = 0

    def callback(msg):
        nonlocal callback_count
        callback_count += 1

    # Test addCallback and removeCallback
    callback_id = queue.addCallback(callback)
    msg = Buffer()
    queue.send(msg)
    time.sleep(0.1)
    assert callback_count == 1

    queue.removeCallback(callback_id)
    queue.send(msg)
    time.sleep(0.1)
    assert callback_count == 1


def test_try_get_all():
    queue = MessageQueue("test", maxSize=10, blocking=True)

    msg1 = Buffer()
    msg2 = Buffer()
    msg3 = Buffer()

    queue.send(msg1)
    queue.send(msg2)
    queue.send(msg3)

    messages = queue.tryGetAll()
    assert len(messages) == 3
    assert messages[0] == msg1
    assert messages[1] == msg2
    assert messages[2] == msg3


def test_get_all():
    queue = MessageQueue("test", maxSize=10, blocking=True)

    msg1 = Buffer()
    msg2 = Buffer()
    msg3 = Buffer()

    def producer():
        time.sleep(0.1)
        queue.send(msg1)
        queue.send(msg2)
        queue.send(msg3)

    thread = threading.Thread(target=producer)
    thread.start()
    thread.join()

    messages = queue.getAll()
    assert len(messages) == 3
    assert messages[0] == msg1
    assert messages[1] == msg2
    assert messages[2] == msg3



def test_close():
    queue = MessageQueue("test", maxSize=10, blocking=True)

    queue.send(Buffer()) # TODO(Morato) - without this line, the get call will block forever, even if the queue is closed
    # Test close
    queue.close()
    assert queue.isClosed()
    with pytest.raises(RuntimeError):
        queue.get()


def test_name_and_properties():
    queue_name = "TestQueue"
    queue = MessageQueue(queue_name, maxSize=10, blocking=False)

    # Test getName
    assert queue.getName() == queue_name

    # Test getMaxSize
    assert queue.getMaxSize() == 10

    # Test getBlocking and setBlocking
    assert not queue.getBlocking()
    queue.setBlocking(True)
    assert queue.getBlocking()

    # Test setMaxSize
    queue.setMaxSize(20)
    assert queue.getMaxSize() == 20


def test_changing_max_size_at_runtime():
    queue = MessageQueue("test", maxSize=10, blocking=True)

    # Fill up the queue
    for _ in range(10):
        msg = Buffer()
        queue.send(msg)

    # Increase maxSize and check if we can send more messages
    queue.setMaxSize(15)
    for _ in range(5):
        msg = Buffer()
        queue.send(msg)
    assert queue.getMaxSize() == 15
    assert queue.getSize() == 15
    assert queue.isFull()

    # Decrease maxSize and check if the queue is truncated
    queue.setMaxSize(5)
    assert queue.getMaxSize() == 5
    messages = [queue.get() for _ in range(5)]
    assert queue.getSize() == 10
    assert queue.isFull()

    # Get all messages
    all_messages = queue.getAll()
    assert len(all_messages) == 10
    assert queue.getSize() == 0
    assert not queue.isFull()

    # Check that 5 messages can be sent and received
    for _ in range(3):
        msg = Buffer()
        queue.send(msg)
    assert queue.getSize() == 3
    assert not queue.isFull()
    for _ in range(3):
        msg = queue.get()
    assert queue.getSize() == 0


def test_adding_and_removing_callbacks_at_runtime():
    queue = MessageQueue("test", maxSize=10, blocking=True)
    callback_count1 = 0
    callback_count2 = 0

    def callback1(msg):
        nonlocal callback_count1
        callback_count1 += 1

    def callback2(msg):
        nonlocal callback_count2
        callback_count2 += 1

    # Add first callback
    callback_id1 = queue.addCallback(callback1)
    msg = Buffer()
    queue.send(msg)
    time.sleep(0.1)
    assert callback_count1 == 1

    # Add second callback
    callback_id2 = queue.addCallback(callback2)
    queue.send(msg)
    time.sleep(0.1)
    assert callback_count1 == 2
    assert callback_count2 == 1

    # Remove first callback
    queue.removeCallback(callback_id1)
    queue.send(msg)
    time.sleep(0.1)
    assert callback_count1 == 2
    assert callback_count2 == 2

    # Remove second callback
    queue.removeCallback(callback_id2)
    queue.send(msg)
    time.sleep(0.1)
    assert callback_count1 == 2
    assert callback_count2 == 2


def test_multi_threaded_callbacks():
    queue = MessageQueue("test", maxSize=10, blocking=True)
    callback_count = 0
    num_callbacks = 10

    def callback(msg):
        nonlocal callback_count
        callback_count += 1

    # Add multiple callbacks from different threads
    def add_callback():
        queue.addCallback(callback)

    callback_threads = [
        threading.Thread(target=add_callback) for _ in range(num_callbacks)
    ]
    for thread in callback_threads:
        thread.start()
    for thread in callback_threads:
        thread.join()

    # Send messages and check if all callbacks are invoked
    msg = Buffer()
    queue.send(msg)
    time.sleep(0.1)
    assert callback_count == num_callbacks



def test_callbacks_with_blocking_queue():
    class CallbackCounter:
        def __init__(self):
            self.count = 0

        def increment(self, msg):
            self.count += 1
    queue_size = 5
    queue = MessageQueue("test", maxSize=queue_size, blocking=True)  # Create a blocking queue
    callback_counter = CallbackCounter()

    # Fill up the queue to its maximum capacity
    for _ in range(queue_size):
        msg = Buffer()
        queue.send(msg)

    assert queue.isFull()

    # Add callbacks
    num_callbacks = 3
    for _ in range(num_callbacks):
        queue.addCallback(callback_counter.increment)

    # Send a message
    success = queue.trySend(Buffer())
    # Check if all callbacks were called
    assert callback_counter.count == num_callbacks, f"Expected {num_callbacks}, got {callback_counter.count}"
    assert not success

    # Consume messages from the queue
    def consume_thread():
        for _ in range(queue_size):
            msg = queue.get()

    thread = threading.Thread(target=consume_thread)
    thread.start()
    thread.join()



def test_callbacks_on_empty_unblocking_queue_then_send_message():
    queue = MessageQueue(
        "test", maxSize=0, blocking=False
    )  # Create a non-blocking queue
    callback_count = 0
    def callback(msg):
        nonlocal callback_count
        callback_count += 1

    # Add a callback
    queue.addCallback(callback)

    # Send a message
    msg = Buffer()
    queue.send(msg)

    assert queue.getSize() == 0
    # Check if the callback was called
    assert callback_count == 1
