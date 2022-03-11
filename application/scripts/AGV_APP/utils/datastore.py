from flask_security import UserMixin as BaseUserMixin

class UserMixin(BaseUserMixin):

    def can_order(self):
        return self.order_store.orderable

    def get_current_order_id(self):
        return self.order_store.current_order
    '''
    def get_order_history(self):
        return self.order_store.order_history
    '''

class Datastore(object):
    def __init__(self, db):
        self.db = db

    def commit(self):
        self.db.session.commit()

    def put(self, model):
        self.db.session.add(model)
        return model


class OrderDatastore(Datastore):

    def __init__(self, db, user_model, store_model, order_model):
        Datastore.__init__(self, db)
        self.user_model = user_model
        self.store_model = store_model
        self.order_model = order_model

    def create_user_order_store(self, user):
        new_order_store = self.store_model(user_id=user.id, orderable=True)
        user.order_store = new_order_store
        self.put(user)
        self.put(new_order_store)

    def create_new_order(self, **kwargs):
        new_order = self.order_model(**kwargs)
        self.put(new_order)
        return new_order

    def add_order_to_user(self, user, order):
        user.order_store.current_order = order.id
        user.order_store.orderable = False
        #user.order_store.order_history.append(order)
        self.put(user.order_store)

    def change_current_order_status(self,user,status):
        current_order = user.get_current_order_id()
        current_order.order_status = status
        self.put(current_order)

    def change_user_order_status(self,user,boolean):
        user.order_store.orderable = boolean
        self.put(user.order_store)


class MerchandiseManager(Datastore):
    def __init__(self, db, merchandise_model):
        Datastore.__init__(self, db)
        self.merchandise_model = merchandise_model

    def find_or_add_merchandise(self, name, **kwargs):
        kwargs["name"] = name
        return self.get_merchandise(name) or self.add_new_merchandise(**kwargs)

    def get_merchandise(self, name):
        return self.merchandise_model.query.filter_by(name=name).first()

    def add_new_merchandise(self, **kwargs):
        merchandise = self.merchandise_model(**kwargs)
        merchandise = self.put(merchandise)
        return merchandise

    '''
    def delete_merchandise(self, merchandise):
        # delete logic goes here
    
    def set_merchandise_num(self, merchandise, int):
        merchandise.num = int
    
    '''
