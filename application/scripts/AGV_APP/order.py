from datetime import datetime

from flask import Blueprint, render_template, redirect, url_for, request
from flask_login import login_required, current_user
from .models import db, Order, OrderStatus, Position
import uuid, queue
from .forms import OrderForm, ConfirmForm
from . import order_datastore, merchandise_manager, service, position_manager
# from . import order_datastore, merchandise_manager,position_manager

order_queue = queue.Queue()
agv_occupied = False

# Blueprint Configuration
order_bp = Blueprint(
    'order_bp', __name__,
    template_folder='templates',
    static_folder='static'
)


@order_bp.route('/create_order', methods=['GET', 'POST'])
@login_required
def create_order():
    # pub.publish("message sent")
    # Create Order logic goes here
    form = OrderForm()
    # User sign-up logic
    if form.validate_on_submit():
        if not current_user.can_order():
            return redirect(url_for('auth_bp.fail'))
        # find the ordered item
        ordered_item = merchandise_manager.get_merchandise(name=request.form.get('merchandise_name'))
        # create and register new order
        if ordered_item is None:
            return redirect(url_for('auth_bp.fail'))  # TODO: Change this for invalid order
        new_order = order_datastore.create_new_order(
            id=uuid.uuid4(),
            merchandise_id=ordered_item.id,
            order_status=OrderStatus.PENDING,
            ordered_time=datetime.now()
        )
        order_datastore.add_order_to_user(current_user, new_order)
        # Commit to database
        db.session.commit()
        # TODO :Add order to Order Queue
        # order_queue.put(new_order.id)
        return redirect(url_for('order_bp.confirm_order'))
    return render_template('create_order.html', form=form)


@order_bp.route('/confirm_order', methods=['GET', 'POST'])
@login_required
def confirm_order():
    form = ConfirmForm()
    if request.method == 'POST':
        order_datastore.change_current_order_status(current_user, OrderStatus.COMPLETED)
        order_datastore.change_user_order_status(current_user, True)
        db.session.commit()
        publish_goal(position_manager.find_origin())
        return redirect(url_for('main_bp.dashboard'))
    goal = current_user.position
    publish_goal(goal)
    return render_template('confirm_order.html', form=form)  # TODO: Add a button for confirm


@order_bp.route('/delete_order')
@login_required
def delete_order():
    current_order = order_datastore.get_user_current_order(current_user)
    order_datastore.delete_order_from_user(current_user, current_order)
    order_datastore.delete_order(current_order)
    db.session.commit()
    return


@order_bp.route('/change_order')
@login_required
def change_order():
    form = OrderForm()
    # User sign-up logic
    if form.validate_on_submit():
        current_order = order_datastore.get_user_current_order()
        if current_order.status == OrderStatus.DELIVERING:
            return redirect(url_for('auth_bp.fail'))
        delete_order()
        # find the ordered item
        ordered_item = merchandise_manager.get_merchandise(name=request.form.get('merchandise_name'))
        # create and register new order
        if ordered_item is None:
            return redirect(url_for('auth_bp.fail'))  # TODO: Change this for invalid order
        new_order = order_datastore.create_new_order(
            id=uuid.uuid4(),
            merchandise_id=ordered_item.id,
            order_status=OrderStatus.PENDING,
            ordered_time=datetime.now()
        )
        order_datastore.add_order_to_user(current_user, new_order)
        # Commit to database
        db.session.commit()
        # TODO :Add order to Order Queue
        order_queue.put(new_order.id)
        return redirect(url_for('order_bp.confirm_order'))
    return render_template('create_order.html', form=form)


# TODO: implement the following function or find alternatives to achieve similar functionality
'''
def order_manager():
    while True:
        while not order_queue.empty():
            current_order = Order.query.filter_by(id=order_queue.get())
            current_order.status = OrderStatus.DELIVERING
            # Publish some message
            while current_order.status == OrderStatus.DELIVERING:
                continue
            print("current order added")
'''


def publish_goal(goal):
    '''
    A method that call for delivery service in ROS
    Parameters:
        goal: position model
            a position model that contains coordinates
    Returns:
        if the delivery navigation is successfully conducted by ROS
    '''
    try:
        return service(goal.x, goal.y, goal.z, goal.w).succeed
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return False
