"""Initialize app."""
from flask import Flask
from flask_sqlalchemy import SQLAlchemy
from flask_login import LoginManager
from flask_security import Security, SQLAlchemyUserDatastore

db = SQLAlchemy()


login_manager = LoginManager()

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
        from .models import User, Role
        from .forms import LoginForm

        # Register Blueprints
        app.register_blueprint(routes.main_bp)
        app.register_blueprint(auth.auth_bp)

        # Initialize Security
        user_datastore = SQLAlchemyUserDatastore(db, User, Role)
        security = Security(app, user_datastore, login_form=LoginForm)

        # Create Database Models
        db.create_all()

        return app
