import React, { useState, useEffect } from 'react';
import { authService, User } from '../../services/authService';
import { useAuth } from '../../contexts/AuthContext';

interface ProfileFormData {
  educationLevel: string;
  programmingExperience: string;
  roboticsBackground: string;
}

interface ProfileFormProps {
  onSuccess?: () => void;
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

const ProfileForm: React.FC<ProfileFormProps> = ({ onSuccess }) => {
  const { user, updateProfile } = useAuth();
  const [formData, setFormData] = useState<ProfileFormData>({
    educationLevel: '',
    programmingExperience: '',
    roboticsBackground: '',
  });

  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);
  const [errors, setErrors] = useState<Record<string, string>>({});

  // Load current profile data when component mounts
  useEffect(() => {
    if (user) {
      setFormData({
        educationLevel: user.educationLevel || '',
        programmingExperience: user.programmingExperience || '',
        roboticsBackground: user.roboticsBackground || '',
      });
    }
  }, [user]);

  const validateForm = (): boolean => {
    const newErrors: Record<string, string> = {};

    if (!formData.educationLevel) {
      newErrors.educationLevel = 'Education level is required';
    }

    if (!formData.programmingExperience) {
      newErrors.programmingExperience = 'Programming experience is required';
    }

    if (!formData.roboticsBackground) {
      newErrors.roboticsBackground = 'Robotics background is required';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));

    // Clear error for this field when user starts typing
    if (errors[name as keyof typeof errors]) {
      setErrors(prev => {
        const newErrors = { ...prev };
        delete newErrors[name];
        return newErrors;
      });
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validateForm()) {
      return;
    }

    setLoading(true);
    setError(null);
    setSuccess(null);

    try {
      await updateProfile(formData);
      setSuccess('Profile updated successfully!');
      if (onSuccess) {
        onSuccess();
      }
    } catch (err: any) {
      setError(err.message || 'An error occurred while updating profile');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="profile-form-container">
      <h2>Update Learning Preferences</h2>

      {error && <div className="error-message">{error}</div>}
      {success && <div className="success-message">{success}</div>}

      <form onSubmit={handleSubmit} className="profile-form">
        <div className="form-group">
          <label htmlFor="educationLevel">Education Level</label>
          <select
            id="educationLevel"
            name="educationLevel"
            value={formData.educationLevel}
            onChange={handleChange}
            className={`form-select ${errors.educationLevel ? 'error' : ''}`}
          >
            <option value="">Select your education level</option>
            {educationLevels.map(level => (
              <option key={level} value={level}>{level}</option>
            ))}
          </select>
          {errors.educationLevel && <div className="error-text">{errors.educationLevel}</div>}
        </div>

        <div className="form-group">
          <label htmlFor="programmingExperience">Programming Experience</label>
          <select
            id="programmingExperience"
            name="programmingExperience"
            value={formData.programmingExperience}
            onChange={handleChange}
            className={`form-select ${errors.programmingExperience ? 'error' : ''}`}
          >
            <option value="">Select your programming experience</option>
            {programmingExperiences.map(exp => (
              <option key={exp} value={exp}>{exp}</option>
            ))}
          </select>
          {errors.programmingExperience && <div className="error-text">{errors.programmingExperience}</div>}
        </div>

        <div className="form-group">
          <label htmlFor="roboticsBackground">Robotics Background</label>
          <select
            id="roboticsBackground"
            name="roboticsBackground"
            value={formData.roboticsBackground}
            onChange={handleChange}
            className={`form-select ${errors.roboticsBackground ? 'error' : ''}`}
          >
            <option value="">Select your robotics background</option>
            {roboticsBackgrounds.map(bg => (
              <option key={bg} value={bg}>{bg}</option>
            ))}
          </select>
          {errors.roboticsBackground && <div className="error-text">{errors.roboticsBackground}</div>}
        </div>


        <button
          type="submit"
          disabled={loading}
          className="auth-button"
        >
          {loading ? 'Updating Profile...' : 'Update Profile'}
        </button>
      </form>
    </div>
  );
};

export default ProfileForm;