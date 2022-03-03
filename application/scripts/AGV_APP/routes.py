"""Logged-in page routes."""
from flask import Blueprint, render_template, redirect, url_for, session
from flask_security import current_user, login_required, logout_user, roles_accepted
from . import pub

# Blueprint Configuration
main_bp = Blueprint(
    'main_bp', __name__,
    template_folder='templates',
    static_folder='static'
)


@main_bp.route('/', methods=['GET'])
@login_required
def dashboard():
    """Logged-in User Dashboard."""
    return render_template('dashboard.html')


# simulate a page that requires admin user-type
@main_bp.route('/admin_page', methods=['GET'])
@roles_accepted("admin")
def admin_page():
    return render_template('admin.html')


@main_bp.route('/logout')
@login_required
def logout():
    """User log-out logic."""
    session.pop('user_id', None)
    logout_user()
    return redirect(url_for('auth_bp.login'))


@main_bp.route('/pub')
@login_required
def publish():
    pub.publish("message sent")
    return redirect(url_for('main_bp.dashboard'))
