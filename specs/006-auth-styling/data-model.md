# Data Model: Auth Component Styling

## UI Components

### Auth Components
- **LoginForm**: Email/password login form with validation states
- **SignupForm**: Registration form with name, email, password, and preference dropdowns
- **ProfileForm**: Form for updating user preferences (3 dropdowns)
- **AuthPage**: Page with toggle between login/signup forms
- **ProfilePage**: Page displaying user info and profile form

### Navbar Component
- **NavbarProfileButton**: Conditional display component showing either profile icon (logged in) or login link (logged out)

## State Requirements
- Authentication status (from useAuth hook)
- Form states (valid/invalid, loading, success/error messages)
- User profile data (name, email, preferences)

## Styling Properties
- CSS Module classes for each component
- Infima variable usage for consistent theming
- Responsive design properties
- Dark mode compatibility

## No Database Changes
This feature only involves UI styling and requires no changes to the existing PostgreSQL or Qdrant schemas.