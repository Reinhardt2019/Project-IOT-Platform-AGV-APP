"""Initialize app."""
from flask import Flask, session
from flask_login import LoginManager
from flask_security import Security, SQLAlchemyUserDatastore
from .database import db
from .models import User, Role
from datetime import timedelta
import uuid
from sqlalchemy_utils import UUIDType

import rospy
import threading
from std_msgs.msg import String


roles_users = db.Table('roles_users',
                       db.Column('user_id', UUIDType(binary=False), db.ForeignKey('user.id'), default=uuid.uuid4,),
                       db.Column('role_id', db.Integer(), db.ForeignKey('role.id')))

login_manager = LoginManager()
user_datastore = SQLAlchemyUserDatastore(db, User, Role)

threading.Thread(target=lambda: rospy.init_node('test_node', disable_signals=True)).start()
pub = rospy.Publisher('/delivery_server/delivery', String, queue_size=1)

def create_app():
    """Construct the core app object."""
    app = Flask(__name__, instance_relative_config=False)

    # Application Configuration
    app.config.from_object('config.Config')

    # Initialize Plugins
    db.init_app(app)
    login_manager.init_app(app)

    with app.app_context():
        from . import routes
        from . import auth
        from .forms import LoginForm,ForgotPasswordForm

        # Register Blueprints
        app.register_blueprint(routes.main_bp)
        app.register_blueprint(auth.auth_bp)

        # Initialize Security and override forms
        security = Security(app,
                            user_datastore,
                            login_form=LoginForm,
                            forgot_password_form=ForgotPasswordForm
                            )

        # Create Database Models
        db.create_all()

        @app.before_first_request
        def before_first_request():
            # Create roles if not exist in database
            admin_role = user_datastore.find_or_create_role(id=1, name="admin",
                                                            description="Administrator for the App.")
            manager_role = user_datastore.find_or_create_role(id=2, name="manager", description="Manager for the App.")
            operator_role = user_datastore.find_or_create_role(id=3, name="operator",
                                                               description="Operator for the App.")
            db.session.commit()

            # Add the unique admin account if not created
            admin_user = user_datastore.find_user(username="admin")
            if admin_user is None:
                admin_user = user_datastore.create_user(id=uuid.uuid4(), username="admin", password="admin", position=0)
                db.session.commit()
            # Add admin role to the unique account
            if not admin_user.has_role(admin_role):
                user_datastore.add_role_to_user(admin_user, admin_role)
                db.session.commit()

        # Set timeout to auto-logout user for inactivity
        @app.before_request
        def before_request():
            session.permanent = True
            app.permanent_session_lifetime = timedelta(minutes=20)
            session.modified = True

        return app
