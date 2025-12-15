import React, { useState } from 'react';
import { authService } from '../../services/authService';

interface FormData {
  email: string;
  password: string;
  name: string;
  educationLevel: string;
  programmingExperience: string;
  roboticsBackground: string;
}

const educationLevels = [
  'High School',
  'Undergraduate',
  'Graduate',
  'Professional'
];

const programmingExperiences = [
  'No Experience',
  'Beginner',
  'Intermediate',
  'Advanced'
];

const roboticsBackgrounds = [
  'No Experience',
  'Hobbyist',
  'Academic',
  'Professional'
];

const SignupForm: React.FC<{ onSuccess: () => void }> = ({ onSuccess }) => {
  const [formData, setFormData] = useState<FormData>({
    email: '',
    password: '',
    name: '',
    educationLevel: 'Undergraduate',
    programmingExperience: 'Beginner',
    roboticsBackground: 'No Experience'
  });

  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    try {
      await authService.signup({
        email: formData.email,
        password: formData.password,
        name: formData.name,
        educationLevel: formData.educationLevel,
        programmingExperience: formData.programmingExperience,
        roboticsBackground: formData.roboticsBackground
      });

      onSuccess();
    } catch (err: any) {
      setError(err.message || 'An error occurred during signup');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="signup-form-container">
      <h2>Create Account</h2>
      {error && <div className="error-message">{error}</div>}

      <form onSubmit={handleSubmit} className="auth-form">
        <div className="form-group">
          <label htmlFor="name">Full Name</label>
          <input
            type="text"
            id="name"
            name="name"
            value={formData.name}
            onChange={handleChange}
            required
            className="form-input"
          />
        </div>

        <div className="form-group">
          <label htmlFor="email">Email</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            required
            className="form-input"
          />
        </div>

        <div className="form-group">
          <label htmlFor="password">Password</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            required
            minLength={8}
            className="form-input"
          />
        </div>

        <div className="form-group">
          <label htmlFor="educationLevel">Education Level</label>
          <select
            id="educationLevel"
            name="educationLevel"
            value={formData.educationLevel}
            onChange={handleChange}
            className="form-select"
          >
            {educationLevels.map(level => (
              <option key={level} value={level}>{level}</option>
            ))}
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="programmingExperience">Programming Experience</label>
          <select
            id="programmingExperience"
            name="programmingExperience"
            value={formData.programmingExperience}
            onChange={handleChange}
            className="form-select"
          >
            {programmingExperiences.map(exp => (
              <option key={exp} value={exp}>{exp}</option>
            ))}
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="roboticsBackground">Robotics Background</label>
          <select
            id="roboticsBackground"
            name="roboticsBackground"
            value={formData.roboticsBackground}
            onChange={handleChange}
            className="form-select"
          >
            {roboticsBackgrounds.map(bg => (
              <option key={bg} value={bg}>{bg}</option>
            ))}
          </select>
        </div>

        <button
          type="submit"
          disabled={loading}
          className="auth-button"
        >
          {loading ? 'Creating Account...' : 'Sign Up'}
        </button>
      </form>
    </div>
  );
};

export default SignupForm;