/**
 * Multi-Agent UI Architecture - Central Export
 *
 * Exports all agents and hooks for UI state management.
 * Provides a clean, modular interface for intelligent UI components.
 */

export { UIStateAgent, uiStateAgent } from './UIStateAgent';
export type { UIState, UIAction } from './UIStateAgent';
export {
  useUIAgent,
  useChatbotAgent,
  useThemeAgent,
  useUserAgent,
} from './useUIAgent';
