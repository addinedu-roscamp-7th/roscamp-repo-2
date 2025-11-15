import threading

class SlotInventory:
    _instance = None
    _lock = threading.Lock()

    def __init__(self):
        # 내부 데이터 보호용 락
        self._status_lock = threading.Lock()
       
        # 슬롯 상태 (True = 채워짐 / False = 비어있음)
        self.slot_status = {
            0: None,
            1: None,
            2: None,
        }
        
        self.book_to_shelf = {
            1: 21,
            2: 22,
            3: 23,
            4: 24,
            5: 25,
            6: 26
        }

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = SlotInventory()
        return cls._instance

    def find_empty_slot(self):
        with self._status_lock:
            for slot_id, book_id in self.slot_status.items():
                if book_id is None:
                    return slot_id
            return None

    def add_book(self, slot_id, book_id):
        with self._status_lock:
            if self.slot_status[slot_id] is not None:
                print(f"⚠️ Slot {slot_id} 이미 책 {self.slot_status[slot_id]} 이 있습니다!")
                return
            self.slot_status[slot_id] = book_id
            print(f"📚 책 {book_id} 을(를) Slot {slot_id} 에 놓았습니다.")
    
    def get_slot_by_book(self, book_id):
        with self._status_lock:
            for slot_id, current_book in self.slot_status.items():
                if current_book == book_id:
                    return slot_id
            return None
    
    def remove_book(self, slot_id):
        with self._status_lock:
            if self.slot_status[slot_id] is None:
                print(f"⚠️ Slot {slot_id} 은 이미 비어 있습니다.")
                return
            book_id = self.slot_status[slot_id]
            self.slot_status[slot_id] = None
            print(f"📦 책 {book_id} 을(를) Slot {slot_id} 에서 꺼냈습니다.")
    
    def get_shelf_by_book(self, book_id):
        with self._status_lock:
            return self.book_to_shelf.get(book_id, None)
    
    def get_book_by_shelf(self, shelf_id):
        with self._status_lock:
            for book_id, shelf in self.book_to_shelf.items():
                if shelf == shelf_id:
                    return book_id
            print(f"❌ [get_book_by_shelf] Shelf {shelf_id}에 대응하는 책이 없습니다.")
            return None
