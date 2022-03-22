

class Datastore(object):
    """
    A Class with SQLAlchemy interfaces
    """

    def __init__(self, db):
        self.db = db

    def commit(self):
        self.db.session.commit()

    def put(self, model):
        self.db.session.add(model)
        return model

    def delete(self, model):
        self.db.session.delete(model)


class OrderDatastore(Datastore):
    """
    A SQLAlchemy datastore implementation for storing user order information
    """

    def __init__(self, db, user_model, store_model, order_model):
        Datastore.__init__(self, db)
        self.user_model = user_model
        self.store_model = store_model
        self.order_model = order_model

    def create_user_order_store(self, user):
        """
        Create an OrderStore for a user and connect it with the user.

        Parameters:
            user: a user model that to link the new order store with
        """
        new_order_store = self.store_model(user_id=user.id, orderable=True)
        self.put(user)
        self.put(new_order_store)

    def create_new_order(self, **kwargs):
        """
        Create a new order with given keyword arguments and put it in the database

        Parameters:
            **kwargs: keyword arguments with should accord with those from ..model/Order
        Returns:
            an order model based on given arguments
        """
        new_order = self.order_model(**kwargs)
        self.put(new_order)
        return new_order

    def get_user_current_order(self, user):
        """
        Get the current order of a user

        Parameters:
            user: user model
                a user model need to be inspected
        Returns:
            an order model that representing the user`s current order
        """
        return self.order_model.query.filter_by(id=user.get_current_order_id()).first()

    def delete_order(self, order):
        """
        Delete an order from the database

        Parameters:
            order: order model
                the order to be deleted
        """
        self.delete(order)

    def add_order_to_user(self, user, order):
        """
        Add an order to a user as their current order

        Parameters:
            user:  user model
                a user model
            order: order model
                an order model to be connected with the given user
        """
        user.order_store.current_order = order.id
        user.order_store.orderable = False
        # user.order_store.order_history.append(order)
        self.put(user.order_store)

    def delete_order_from_user(self, user, order):
        """
        Delete a current order from an user and remove the order from the order history
        ** This function is not completely implemented **
        TODO: implement this funciton
        Parameters:
            user: user model
                the user whose order history need to be changed
            order: order model
                the order to be removed
        """
        user.order_store.current_order = None
        user.order_store.orderable = True
        # user.order_store.order_history.remo(order)
        self.put(user.order_store)

    def change_current_order_status(self, user, status):
        """
        Change the current order status of the user

        Parameters:
            user: user model
                the user whose current order status need to be changed
            status: OrderStatus
                the new status set to the order
        """
        current_order = self.get_user_current_order(user)
        current_order.order_status = status
        self.put(current_order)

    def change_user_order_status(self, user, boolean):
        """
        Change the orderable status for a user
        Parameters:
            user: user model
                the user whose orderable stauts need to be changed
            boolean: boolean
                the user`s new orderable status
        """
        user.order_store.orderable = boolean
        self.put(user.order_store)


class MerchandiseManager(Datastore):
    """
    A SQLAlchemy datastore implementation for managing merchandise information
    """

    def __init__(self, db, merchandise_model):
        Datastore.__init__(self, db)
        self.merchandise_model = merchandise_model

    def add_new_merchandise(self, **kwargs):
        """
        Create a new merchandise based on given keyword arguments.
        Parameters:
            **kwargs: keyword arguments
                attributes that accord with ..model.Merchandise
         Returns:
            the newly created merchandise model
        """
        merchandise = self.merchandise_model(**kwargs)
        merchandise = self.put(merchandise)
        return merchandise

    def get_merchandise(self, name):
        """
        Find a merchandise in the database

        Parameter:
            name: string
                the name of the target merchandise
        Returns:
            the target merchandise model. None if such model does not exist
        """
        return self.merchandise_model.query.filter_by(name=name).first()

    def find_or_add_merchandise(self, name, **kwargs):
        """
        Find a merchandise in the database based on name and other given keyword arguments. Creat a new merchandise if
        such entry is not found.

        Parameters:
            name: string
                the name of the target merchandise
            **kwargs: keyword arguments
                attributes that accord with ..model.Merchandise
        Returns:
            a merchandise model
        """
        kwargs["name"] = name
        return self.get_merchandise(name) or self.add_new_merchandise(**kwargs)

    '''
    TODO: implement the following function
    def delete_merchandise(self, merchandise):
        # delete logic goes here
    
    def set_merchandise_num(self, merchandise, int):
        merchandise.num = int
    
    '''
