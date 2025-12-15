---
id: 6
title: "Add Styling to Auth Components Specification"
stage: "spec"
date_iso: "2025-12-15"
surface: "agent"
model: "claude-opus-4-5-20251101"
feature: "auth-styling"
branch: "006-auth-styling"
user: "user"
command: "/sp.specify Add Styling to Auth Components (Preserve Functionality)"
labels: ["specification", "auth", "styling", "ui", "css-modules"]
links:
  spec: "/mnt/d/F drive backup/coding Q4/hackathon/save-9/Docusaurus/specs/006-auth-styling/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - /mnt/d/F drive backup/coding Q4/hackathon/save-9/Docusaurus/specs/006-auth-styling/spec.md
  - /mnt/d/F drive backup/coding Q4/hackathon/save-9/Docusaurus/specs/006-auth-styling/checklists/requirements.md
tests_yaml: []
prompt_text: |
  name the new branch "006-adding-ccs" and heres is my specs :# Add Styling to Auth Components (Preserve Functionality)

  ## Subject
  Add professional styling to `LoginForm.tsx`, `SignupForm.tsx`, `ProfileForm.tsx`, `auth.tsx`, and `profile.tsx` using CSS Modules while preserving all existing functional logic.

  ---

  ## Context
  This Docusaurus project has five auth-related files that require styling:

  | File | Location | Purpose |
  |------|----------|---------|
  | `LoginForm.tsx` | `frontend/src/components/Auth/` | Email/password login form |
  | `SignupForm.tsx` | `frontend/src/components/Auth/` | Registration with name, email, password + 3 dropdowns |
  | `ProfileForm.tsx` | `frontend/src/components/Auth/` | Update learning preferences (3 dropdowns) |
  | `auth.tsx` | `frontend/src/pages/` | Page with Login/Signup toggle tabs |
  | `profile.tsx` | `frontend/src/pages/` | User info display + ProfileForm |

  **The functional logic (TypeScript hooks, state management, event handlers, API calls) is fully operational.** The UI currently uses raw HTML/JSX with unstyled className attributes.

  ---

  ## Additional Feature: Navbar Profile Button

  Add a profile icon/button to the Docusaurus navbar that:
  - Shows a **user icon** (e.g., a circle with user silhouette or initials)
  - When clicked, navigates to `/profile` page
  - **Conditional display:**
    - If user is **logged in**: Show profile icon + optional user initial/avatar
    - If user is **not logged in**: Show "Login" text link or login icon
  - Position: Right side of navbar, before any existing navbar items
  - Use Docusaurus theme swizzling to customize the navbar

  ### Implementation Approach
  Use Docusaurus **theme swizzling** to add a custom navbar item:
  1. Create `NavbarProfileButton` component in `frontend/src/theme/NavbarItem/`
  2. Use `useAuth` hook to check authentication state
  3. Use `@docusaurus/Link` for navigation
  4. Style with CSS Module using Infima variables

  ### Navbar Profile Button Styling
  | State | Styling |
  |-------|---------|
  | Container | Flex, align-items center, padding 0.5rem |
  | Icon (logged out) | Simple user outline icon, `--ifm-font-color-base` |
  | Avatar (logged in) | 32px circle, primary background, white text initial |
  | Hover | Subtle opacity change or background highlight |

  ---

  ## Crucial Constraint - Functional Integrity

  > [!CAUTION]
  > **DO NOT modify existing logic.** Keep all `useState`, `useEffect`, `onSubmit`, `onClick`, and other event handlers exactly as they are.

  - **DO NOT refactor the code structure** unless wrapping elements for styling.
  - **Task is strictly presentation:** Only update `className` attributes and import CSS modules.

  ---

  ## Technical Requirements

  ### CSS Approach
  Use **CSS Modules**. Create:
  - `Auth.module.css` → for `auth.tsx`, `LoginForm.tsx`, `SignupForm.tsx`
  - `Profile.module.css` → for `profile.tsx`, `ProfileForm.tsx`

  ### Docusaurus Integration
  Use **Infima CSS variables** for automatic dark mode support:

  ```css
  var(--ifm-color-primary)              /* Primary theme color */
  var(--ifm-color-primary-dark)         /* Hover states */
  var(--ifm-background-color)           /* Page background */
  var(--ifm-background-surface-color)   /* Card/elevated surfaces */
  var(--ifm-color-emphasis-300)         /* Borders */
  var(--ifm-font-color-base)            /* Text color */
  var(--ifm-border-radius)              /* Consistent rounding */
  var(--ifm-spacing-horizontal)         /* Spacing */
  ```

  ### Responsive Design
  Ensure mobile-friendly layouts with appropriate breakpoints.

  ---

  ## Existing CSS Classes to Style

  ### Auth Page & Forms (`auth.tsx`, `LoginForm.tsx`, `SignupForm.tsx`)
  ```
  .auth-page              - Full page wrapper
  .auth-container         - Card container
  .auth-toggle            - Login/Signup tab container
    button.active         - Active tab state
  .login-form-container   - Login form wrapper
  .signup-form-container  - Signup form wrapper
  .auth-form              - Form element
  .form-group             - Label + input wrapper
  .form-input             - Text inputs
  .form-select            - Dropdown selects
  .auth-button            - Submit button
  .error-message          - Error alert
  ```

  ### Profile Page & Form (`profile.tsx`, `ProfileForm.tsx`)
  ```
  .profile-page           - Full page wrapper
  .profile-container      - Card container
  .profile-header         - Title + description section
  .profile-info           - User details section
  .profile-details        - Details grid/list
  .profile-loading        - Loading state
  .profile-auth-required  - "Please login" block
  .profile-form-container - Form wrapper
  .profile-form           - Form element
  .profile-form-section   - Form section wrapper
  .success-message        - Success alert
  .error-text             - Field-level error text
  ```

  ---

  ## Design Specifications

  ### Layout
  | Element | Specification |
  |---------|---------------|
  | Auth Card | Max-width: 420px, centered, rounded corners (8px), subtle shadow |
  | Profile Card | Max-width: 600px, centered |
  | Mobile | Full-width with 1rem padding |

  ### Form Elements
  | Element | Styling |
  |---------|---------|
  | Inputs/Selects | Full-width, 12px vertical padding, 16px horizontal, 1px border using `--ifm-color-emphasis-300` |
  | Focus State | Primary color ring/border using `--ifm-color-primary` |
  | Labels | Bold, small margin-bottom |

  ### Buttons
  | State | Styling |
  |-------|---------|
  | Default | Primary background, white text, full-width |
  | Hover | Darker primary (`--ifm-color-primary-dark`) |
  | Disabled | Opacity 0.6, cursor not-allowed |

  ### Toggle Tabs
  | State | Styling |
  |-------|---------|
  | Container | Flex row, gap, margin-bottom |
  | Inactive | Transparent bg, subtle text |
  | Active | Primary color underline or background |

  ### Feedback Messages
  | Type | Color |
  |------|-------|
  | Error | Red background with dark red text |
  | Success | Green background with dark green text |

  ### Profile Info Grid
  | Layout | Specification |
  |--------|---------------|
  | Desktop | Two-column grid |
  | Mobile | Single column stack |
  | Items | Label + value pairs with subtle separators |

  ---

  ## Design Aesthetic
  - **Style:** Clean, minimalist, accessible
  - **Spacing:** Generous padding (1.5rem card padding, 1rem between form groups)
  - **Feedback:** Hover effects on buttons, focus rings on inputs
  - **Consistency:** Match Docusaurus site theme colors

  ---

  ## Files to Create
  ```
  frontend/src/components/Auth/Auth.module.css
  frontend/src/pages/Profile.module.css
  frontend/src/theme/NavbarItem/NavbarProfileButton.tsx
  frontend/src/theme/NavbarItem/NavbarProfileButton.module.css
  ```

  ## Files to Modify
  ```
  frontend/src/pages/auth.tsx                   → Import Auth.module.css
  frontend/src/components/Auth/LoginForm.tsx    → Import Auth.module.css
  frontend/src/components/Auth/SignupForm.tsx   → Import Auth.module.css
  frontend/src/pages/profile.tsx                → Import Profile.module.css
  frontend/src/components/Auth/ProfileForm.tsx  → Import Profile.module.css
  frontend/docusaurus.config.js                 → Add custom navbar item configuration
  ```
response_text: |
  Created specification for adding styling to auth components while preserving functionality. The spec includes:
  - Professional styling for LoginForm.tsx, SignupForm.tsx, ProfileForm.tsx, auth.tsx, and profile.tsx
  - CSS Module implementation with Infima variables for dark mode support
  - Navbar profile button with conditional display based on authentication status
  - Preservation of all existing functionality
  - Comprehensive user scenarios, functional requirements, and success criteria
outcome: |
  Specification created successfully with branch 006-auth-styling. The spec is ready for planning phase.
evaluation: |
  The specification meets all quality criteria: no implementation details, testable requirements, measurable success criteria, and comprehensive coverage of the feature requirements.
---

# Add Styling to Auth Components (Preserve Functionality)

## Subject
Add professional styling to `LoginForm.tsx`, `SignupForm.tsx`, `ProfileForm.tsx`, `auth.tsx`, and `profile.tsx` using CSS Modules while preserving all existing functional logic.

---

## Context
This Docusaurus project has five auth-related files that require styling:

| File | Location | Purpose |
|------|----------|---------|
| `LoginForm.tsx` | `frontend/src/components/Auth/` | Email/password login form |
| `SignupForm.tsx` | `frontend/src/components/Auth/` | Registration with name, email, password + 3 dropdowns |
| `ProfileForm.tsx` | `frontend/src/components/Auth/` | Update learning preferences (3 dropdowns) |
| `auth.tsx` | `frontend/src/pages/` | Page with Login/Signup toggle tabs |
| `profile.tsx` | `frontend/src/pages/` | User info display + ProfileForm |

**The functional logic (TypeScript hooks, state management, event handlers, API calls) is fully operational.** The UI currently uses raw HTML/JSX with unstyled className attributes.

---

## Additional Feature: Navbar Profile Button

Add a profile icon/button to the Docusaurus navbar that:
- Shows a **user icon** (e.g., a circle with user silhouette or initials)
- When clicked, navigates to `/profile` page
- **Conditional display:**
  - If user is **logged in**: Show profile icon + optional user initial/avatar
  - If user is **not logged in**: Show "Login" text link or login icon
- Position: Right side of navbar, before any existing navbar items
- Use Docusaurus theme swizzling to customize the navbar

### Implementation Approach
Use Docusaurus **theme swizzling** to add a custom navbar item:
1. Create `NavbarProfileButton` component in `frontend/src/theme/NavbarItem/`
2. Use `useAuth` hook to check authentication state
3. Use `@docusaurus/Link` for navigation
4. Style with CSS Module using Infima variables

### Navbar Profile Button Styling
| State | Styling |
|-------|---------|
| Container | Flex, align-items center, padding 0.5rem |
| Icon (logged out) | Simple user outline icon, `--ifm-font-color-base` |
| Avatar (logged in) | 32px circle, primary background, white text initial |
| Hover | Subtle opacity change or background highlight |

---

## Crucial Constraint - Functional Integrity

> [!CAUTION]
> **DO NOT modify existing logic.** Keep all `useState`, `useEffect`, `onSubmit`, `onClick`, and other event handlers exactly as they are.

- **DO NOT refactor the code structure** unless wrapping elements for styling.
- **Task is strictly presentation:** Only update `className` attributes and import CSS modules.

---

## Technical Requirements

### CSS Approach
Use **CSS Modules**. Create:
- `Auth.module.css` → for `auth.tsx`, `LoginForm.tsx`, `SignupForm.tsx`
- `Profile.module.css` → for `profile.tsx`, `ProfileForm.tsx`

### Docusaurus Integration
Use **Infima CSS variables** for automatic dark mode support:

```css
var(--ifm-color-primary)              /* Primary theme color */
var(--ifm-color-primary-dark)         /* Hover states */
var(--ifm-background-color)           /* Page background */
var(--ifm-background-surface-color)   /* Card/elevated surfaces */
var(--ifm-color-emphasis-300)         /* Borders */
var(--ifm-font-color-base)            /* Text color */
var(--ifm-border-radius)              /* Consistent rounding */
var(--ifm-spacing-horizontal)         /* Spacing */
```

### Responsive Design
Ensure mobile-friendly layouts with appropriate breakpoints.

---

## Existing CSS Classes to Style

### Auth Page & Forms (`auth.tsx`, `LoginForm.tsx`, `SignupForm.tsx`)
```
.auth-page              - Full page wrapper
.auth-container         - Card container
.auth-toggle            - Login/Signup tab container
  button.active         - Active tab state
.login-form-container   - Login form wrapper
.signup-form-container  - Signup form wrapper
.auth-form              - Form element
.form-group             - Label + input wrapper
.form-input             - Text inputs
.form-select            - Dropdown selects
.auth-button            - Submit button
.error-message          - Error alert
```

### Profile Page & Form (`profile.tsx`, `ProfileForm.tsx`)
```
.profile-page           - Full page wrapper
.profile-container      - Card container
.profile-header         - Title + description section
.profile-info           - User details section
.profile-details        - Details grid/list
.profile-loading        - Loading state
.profile-auth-required  - "Please login" block
.profile-form-container - Form wrapper
.profile-form           - Form element
.profile-form-section   - Form section wrapper
.success-message        - Success alert
.error-text             - Field-level error text
```

---

## Design Specifications

### Layout
| Element | Specification |
|---------|---------------|
| Auth Card | Max-width: 420px, centered, rounded corners (8px), subtle shadow |
| Profile Card | Max-width: 600px, centered |
| Mobile | Full-width with 1rem padding |

### Form Elements
| Element | Styling |
|---------|---------|
| Inputs/Selects | Full-width, 12px vertical padding, 16px horizontal, 1px border using `--ifm-color-emphasis-300` |
| Focus State | Primary color ring/border using `--ifm-color-primary` |
| Labels | Bold, small margin-bottom |

### Buttons
| State | Styling |
|-------|---------|
| Default | Primary background, white text, full-width |
| Hover | Darker primary (`--ifm-color-primary-dark`) |
| Disabled | Opacity 0.6, cursor not-allowed |

### Toggle Tabs
| State | Styling |
|-------|---------|
| Container | Flex row, gap, margin-bottom |
| Inactive | Transparent bg, subtle text |
| Active | Primary color underline or background |

### Feedback Messages
| Type | Color |
|------|-------|
| Error | Red background with dark red text |
| Success | Green background with dark green text |

### Profile Info Grid
| Layout | Specification |
|--------|---------------|
| Desktop | Two-column grid |
| Mobile | Single column stack |
| Items | Label + value pairs with subtle separators |

---

## Design Aesthetic
- **Style:** Clean, minimalist, accessible
- **Spacing:** Generous padding (1.5rem card padding, 1rem between form groups)
- **Feedback:** Hover effects on buttons, focus rings on inputs
- **Consistency:** Match Docusaurus site theme colors

---

## Files to Create
```
frontend/src/components/Auth/Auth.module.css
frontend/src/pages/Profile.module.css
frontend/src/theme/NavbarItem/NavbarProfileButton.tsx
frontend/src/theme/NavbarItem/NavbarProfileButton.module.css
```

## Files to Modify
```
frontend/src/pages/auth.tsx                   → Import Auth.module.css
frontend/src/components/Auth/LoginForm.tsx    → Import Auth.module.css
frontend/src/components/Auth/SignupForm.tsx   → Import Auth.module.css
frontend/src/pages/profile.tsx                → Import Profile.module.css
frontend/src/components/Auth/ProfileForm.tsx  → Import Profile.module.css
frontend/docusaurus.config.js                 → Add custom navbar item configuration