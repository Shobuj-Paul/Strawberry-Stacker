#Consider there are N items in the lab initially. Therefore create a list L that will 
#contain all the names of items (item_name) along with its quantity (item_quantity).

#Also, make provision for adding/deleting items to/from the list. The item name and 
#its quantity along with the operation (ADD / DELETE) will be specified. You must take 
#care of the following conditions while performing any of the operations on the list.

class Inventory:
    def __init__(self, items = [], count = []):
        self.items = items
        self.count = count
    def add_items(self, item_name, item_count):
        if item_name not in self.items:
            self.items.append(item_name)
            self.count.append(item_count)
            print("ADDED Item {}".format(item_name))
        else:
            index = self.items.index(item_name)
            self.count[index] = self.count[index] + item_count
            print("UPDATED Item {}".format(item_name))
    def delete_items(self, item_name, item_count):
        if item_name not in self.items:
            print("Item {} does not exist".format(item_name))
        else:
            index = self.items.index(item_name)
            if item_count > self.count[index]:
                print("Item {} could not be DELETED".format(item_name))
            elif item_count < self.count[index]:
                self.count[index] = self.count[index] - item_count
                print("DELETED Item {}".format(item_name))
            elif item_count == self.count[index]:
                self.count.pop(index)
                self.items.pop(index)
                print("DELETED Item {}".format(item_name))
    def total_count(self):
        print("Total Items in Inventory:",sum(self.count))

def main():
    T = int(input())
    for _ in range(0,T):
        items = []
        count = []
        oper = []
        oper_item = []
        oper_count = []
        N = int(input())
        for i in range(0,N):
            temp = str(input())
            items.append(temp.split()[0])
            count.append(int(temp.split()[1]))
        count = list(map(int, count))
        M = int(input())
        for i in range(0,M):
            temp = str(input())
            oper.append(temp.split()[0])
            oper_item.append(temp.split()[1])
            oper_count.append(int(temp.split()[2]))
        Ash = Inventory(items, count)
        for i in range(0,M):
            if oper[i] == "ADD":
                Ash.add_items(oper_item[i], oper_count[i])
            elif oper[i] == "DELETE":
                Ash.delete_items(oper_item[i], oper_count[i])
        Ash.total_count()

if __name__=='__main__':
    try: main()
    except: pass