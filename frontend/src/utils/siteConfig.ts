/**
 * Site configuration helper for accessing Docusaurus customFields
 * This provides access to environment-based URLs for services
 * 
 * Usage in React components: Use useDocusaurusContext() hook directly
 * Usage in services: Import getSiteConfig() from this file
 */

// Type definition for our custom fields
export interface SiteCustomFields {
    backendUrl: string;
    authUrl: string;
}

// This will be populated by Docusaurus at runtime
// Fallback values for local development when config isn't available
const defaultConfig: SiteCustomFields = {
    backendUrl: 'http://localhost:8000',
    authUrl: 'http://localhost:3001',
};

/**
 * Get site configuration from Docusaurus
 * For React components, prefer using useDocusaurusContext() hook instead
 */
export function getSiteConfig(): SiteCustomFields {
    // Try to get from global Docusaurus object (available at runtime in browser)
    if (typeof window !== 'undefined' && (window as any).__DOCUSAURUS__) {
        const docusaurus = (window as any).__DOCUSAURUS__;
        if (docusaurus?.siteConfig?.customFields) {
            return {
                backendUrl: docusaurus.siteConfig.customFields.backendUrl || defaultConfig.backendUrl,
                authUrl: docusaurus.siteConfig.customFields.authUrl || defaultConfig.authUrl,
            };
        }
    }

    // Fallback to default config for SSR or when Docusaurus context isn't available
    return defaultConfig;
}

/**
 * Get the backend API URL
 */
export function getBackendUrl(): string {
    return getSiteConfig().backendUrl;
}

/**
 * Get the auth server URL
 */
export function getAuthUrl(): string {
    return getSiteConfig().authUrl;
}
