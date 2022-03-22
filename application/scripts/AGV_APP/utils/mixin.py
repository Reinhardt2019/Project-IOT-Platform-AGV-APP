from flask_security import UserMixin as BaseUserMixin


class UserMixin(BaseUserMixin):
    """
    An extension to UserMixin from flask-security
    """
    def can_order(self):
        """
        return if the user can make new order
        """
        return self.order_store.orderable

    def get_current_order_id(self):
        """
        return the current order id of the user
        """
        id = self.order_store.current_order
        return id

    '''
    TODO: implement this function
    def get_order_history(self):
        return self.order_store.order_history
    '''
