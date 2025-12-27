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
    backendUrl: 'https://ahmedsaeed4-hackathon-1-backend.hf.space',
    authUrl: 'https://physical-ai-humanoid-robotics-1.onrender.com',
};

/**
 * Get site configuration from Docusaurus
 * For React components, prefer using useDocusaurusContext() hook instead
 */
export function getSiteConfig(): SiteCustomFields {
    // DEBUG: Log what's available at runtime (check browser console)
    if (typeof window !== 'undefined') {
        console.log('=== SITECONFIG RUNTIME DEBUG ===');
        console.log('window.__DOCUSAURUS__:', (window as any).__DOCUSAURUS__);
        console.log('customFields:', (window as any).__DOCUSAURUS__?.siteConfig?.customFields);
        console.log('================================');
    }

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
